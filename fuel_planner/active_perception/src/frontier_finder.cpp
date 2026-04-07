#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
// #include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

namespace fast_planner {
/*
  FrontierFinder 是 FUEL 里专门维护 Frontier Information Structure 的模块。

  它在整条探索链路里的职责可以概括成三件事：
  1. 根据最新局部地图，增量删除失效 frontier，增量发现新 frontier
  2. 为每个 frontier cluster 生成可行 viewpoint，并按可见性排序
  3. 维护 frontier 之间的路径 / 代价缓存，供上层全局 tour 规划直接使用

  读这份文件时，建议抓住下面这条主线：
  - searchFrontiers()
    只在“地图更新区域附近”做 frontier 的删旧增新
  - computeFrontiersToVisit()
    给新 frontier 采样 viewpoint，并分成 active / dormant
  - updateFrontierCostMatrix()
    增量维护 active frontiers 之间的 pairwise path / cost

  这也是论文里 frontier information structure 真正落地的地方。
*/
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh) {
  this->edt_env_ = edt;

  /*
    frontier_flag_ 是一个“按体素地址索引”的标记数组。
    运行时语义很重要：

    - 值为 1：这个 voxel 当前已经属于某个已维护的 frontier cluster
      不管这个 cluster 还在 active frontiers_ 里，还是已经被放进 dormant_frontiers_

    - 值为 0：这个 voxel 还没有被任何 frontier cluster 占用

    这样做的目的不是“一轮扫描内去重”那么简单，而是跨轮次保留 frontier 归属关系，
    从而支持增量更新：
    - 未变化的 frontier 不需要重新长一遍
    - 只有被更新区域影响到的 frontier 才会被移除并重新搜索
  */
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  /*
    frontier 参数控制两类事情：
    1. frontier cluster 如何形成 / 何时拆分
    2. viewpoint 在什么范围采样 / 怎样判定“覆盖效果足够好”
  */
  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

  /*
    这里这套 RayCaster 不用于轨迹搜索，而是用于 frontier / viewpoint 相关的可见性判断：
    - 统计候选 viewpoint 能看到多少 frontier cell
    - 按视线检查某个 frontier cell 是否被障碍遮挡
    - 估计某条视线上的 unknown / occupied 穿越情况
  */
  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  // PerceptionUtils 负责 FOV 几何计算，例如“某个点是否落在当前相机视场中”。
  percep_utils_.reset(new PerceptionUtils(nh));
}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::searchFrontiers() {
  /*
    frontier 增量更新的主入口。

    这一步只负责“删旧 + 找新”，不负责 viewpoint 采样。

    整体流程是：
    1. 读取地图最新的 updated box
    2. 删除所有与该 box 重叠且已失效的 frontier
    3. 只在 updated box 附近重新扫描新的 frontier seed
    4. 对新 cluster 做 region growing，并在必要时做大 cluster 拆分

    输出先放到 tmp_frontiers_ 里，后面 computeFrontiersToVisit() 才决定：
    - 哪些进入 active frontiers_
    - 哪些进入 dormant_frontiers_
  */
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // updated box 是 SDFMap 维护的“最近一轮真正发生地图变化的局部包围盒”。
  // FUEL 的增量 frontier 更新正是围绕这个盒子展开，而不是每次全图重扫。
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  /*
    resetFlag 的作用是把一个旧 frontier 彻底从维护结构里移除：
    - 清掉它所有 frontier voxel 在 frontier_flag_ 里的占用标记
    - 从对应 frontiers 列表中擦除

    只有先清标记，后面扫描 updated 区域时这些 voxel 才有机会重新长出新的 frontier cluster。
  */
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  std::cout << "Before remove: " << frontiers_.size() << std::endl;

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    /*
      只有同时满足两件事才删除 active frontier：
      1. 它的包围盒和 update box 有重叠
      2. 它内部至少有一个 voxel 已经不再满足 frontier 条件

      removed_ids_ 记录的是“被删掉的旧 active frontier 在旧顺序中的编号”，
      后面 updateFrontierCostMatrix() 会用它增量删除旧代价矩阵里的对应列。
    */
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    // dormant frontier 也要同步清理，只是它们不参与 cost matrix，因此不需要记录 removed_ids_。
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  /*
    新 frontier 的搜索范围不会严格限制在 update box 本身，而是额外向外扩一圈。
    这是因为：
    - frontier 的定义依赖“free cell 邻接 unknown cell”
    - 一处地图更新可能会连带影响边界附近的 frontier 归属

    因此这里用一个略微膨胀过的 search box 来重新找 seed，会更稳。
  */
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // 逐 voxel 扫描 search box，寻找新的 frontier seed。
        Eigen::Vector3i cur(x, y, z);
        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
          // 一旦发现尚未归属的 frontier seed，就从它做区域生长，扩成完整 cluster。
          expandFrontier(cur);
        }
      }
  // 如果某个新 cluster 在水平方向上过大，就进一步按主方向拆分成多个更紧凑的 frontier。
  splitLargeFrontiers(tmp_frontiers_);

  ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
}

void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {
  /*
    从一个 frontier seed 出发做 region growing，长出完整 frontier cluster。

    这里的 frontier 条件非常直接：
    - 当前 voxel 是 known free
    - 六邻域里至少有一个 unknown

    扩张阶段使用 allNeighbors()，也就是 26 邻域连通，
    这样对斜向相接的 frontier voxel 也会归到同一个 cluster 里。
  */

  // BFS/region-growing 的工作队列与临时结果缓存。
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // 基于 26 邻域做区域生长，把与 seed 连通的 frontier voxel 全部吸进来。
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // 合法邻居需要同时满足：
      // 1. 还没被任何 frontier cluster 标记过
      // 2. 仍在有效地图盒内
      // 3. 自己仍然是 frontier voxel
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.4) continue;  // 去掉靠地面的 frontier 噪声，通常是地面附近建图误差或边缘伪 frontier。
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // 只有 cluster 足够大才保留；太小的 frontier 往往只是噪声或对探索价值很低。
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  /*
    对新找到的 frontier 做一次“是否过大”的检查。
    这一步是 region growing 之后的后处理，目的不是合并，而是把特别长的 frontier 条带拆小，
    避免一个 cluster 横向跨度太大，导致后续 viewpoint 既难选也不稳定。
  */
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // 每个 frontier 单独判断是否需要沿水平主方向继续拆分。
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  /*
    如果一个 frontier 在水平面上过大，就按 PCA 的第一主方向把它二分。

    这里的思路和论文里的 cluster split 是一致的：
    - 先看 cluster 相对中心的横向扩展是否超过阈值
    - 若超过，就在 XY 平面做 PCA
    - 再沿第一主成分方向把点集一分为二
    - 对子块递归重复，直到每块都足够紧凑
  */
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // 在降采样后的 frontier 点云上统计 XY 协方差矩阵，估计主方向。
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // 取最大特征值对应的特征向量，作为水平第一主成分方向。
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  std::cout << "max idx: " << max_idx << std::endl;
  std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

  // 沿第一主成分的法向符号把点分成两组，相当于在 cluster 主轴上切一刀。
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // 如果子块仍然过大，就继续递归拆，直到满足尺度约束为止。
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::isInBoxes(
    const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx) {
  // 辅助工具：判断某个 voxel 是否落在给定的一组 box 内。
  // 当前主流程基本没显式使用它，更多是旧逻辑 / 调试阶段遗留下来的工具函数。
  Vector3d pt;
  edt_env_->sdf_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}

void FrontierFinder::updateFrontierCostMatrix() {
  /*
    增量维护 active frontier 之间的 pairwise path / cost 缓存。

    这是整个 Frontier Information Structure 最核心的缓存更新步骤之一。

    需要先理解两个运行时变量：
    - removed_ids_
      本轮从旧 active frontiers_ 中删掉了哪些 frontier。
      这里记录的是“删除前旧顺序里的编号”，用于从旧 cost/path 列表里删列。

    - first_new_ftr_
      指向本轮新插入 active frontiers_ 的第一个 frontier。
      它把当前 frontiers_ 列表切成两段：
      1. [begin, first_new_ftr_)      : 上一轮就存在、这轮继续保留的 old frontiers
      2. [first_new_ftr_, end)        : 这轮新长出来并成功找到 viewpoint 的 new frontiers

    因此这里不需要每轮重建完整矩阵，而是只做三件事：
    1. 从 surviving old frontiers 的 cost/path 列表里删除被移除 frontier 的列
    2. 计算 old <-> new 之间的新增 cost/path
    3. 计算 new <-> new 之间的新增 cost/path
  */
  std::cout << "cost mat size before remove: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

  std::cout << "cost mat size remove: " << std::endl;
  if (!removed_ids_.empty()) {
    /*
      对仍然存活的 old frontiers 来说，它们内部原本维护着“到所有旧 frontiers 的路径/代价列表”。
      一旦旧 frontier 有删除，就必须把这些列表中对应的位置删掉。

      这里删除的是“列”，不是整行：
      - 外层遍历每个 surviving old frontier，相当于逐行处理
      - 内层按 removed_ids_ 删除该行中对应 frontier 的 cost/path 项
    */
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (int i = 0; i < removed_ids_.size(); ++i) {
        // 把 list iterator 走到本次要删除的那个“旧 frontier 列位置”。
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
      std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
    }
    removed_ids_.clear();
  }
  std::cout << "" << std::endl;

  auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
    std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
    /*
      这里统一用每个 frontier 的 top viewpoint（排序后的第一个 viewpoint）来定义 cluster 间代价。
      也就是说，此处维护的是“粗层级 frontier graph”的边，不是 refinement 之后的多候选 viewpoint 图。
    */
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    /*
      这份缓存是“按 frontier 顺序附在每个 frontier 自己身上”的：
      - it1->costs_ / paths_ 保存从 it1 出发到其它 frontier 的项
      - it2 也要保存一份反向路径，便于后面直接索引
    */
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  std::cout << "cost mat add: " << std::endl;
  // 先补 old <-> new 的边。这部分是典型的增量更新，不动 old-old 的既有缓存。
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
      updateCost(it1, it2);

  // 再补 new <-> new 的边。对角线是自己到自己，代价记 0，路径留空。
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
  std::cout << "" << std::endl;
  std::cout << "cost mat size final: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;
}

void FrontierFinder::mergeFrontiers(Frontier& ftr1, const Frontier& ftr2) {
  // 旧逻辑保留函数：把两个 frontier 直接并成一个，再重新计算平均位置/包围盒/降采样结果。
  ftr1.average_ =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
  computeFrontierInfo(ftr1);
}

bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2) {
  // 判断若把两个 frontier 合并后，是否会超过 cluster 尺度限制。
  // 当前主流程更偏向“拆大块”，这个函数更多是旧思路遗留的工具。
  Vector3d merged_avg =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  return true;
}

bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // 判断两个 axis-aligned bounding box 是否有重叠。
  // 这是 frontier 增量删除时最先做的“粗筛”。
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  /*
    判断一个旧 frontier 是否已经失效。

    判定标准很直接：
    frontier 里的任意一个 cell 只要已经不再满足
    “known free 且六邻域里存在 unknown”
    这个 frontier 就被视为 changed，需要删掉并在 updated box 附近重新搜索。
  */
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  /*
    为一个 frontier cluster 计算后续会反复用到的几何信息：
    - average_       : cluster 几何中心，后续 viewpoint 采样围绕它展开
    - box_min_/max_  : cluster 包围盒，后续用于增量更新重叠判断和可视化
    - filtered_cells_: 降采样后的 frontier 点集，后续用于 viewpoint 评估，降低计算量
  */
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // frontier 原始 cells_ 可能很多，先降采样再用于后面的 viewpoint 采样与可见性统计。
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  /*
    处理本轮 tmp_frontiers_，决定哪些 frontier 真正进入“待访问集合”。

    这里做的不是 frontier 搜索，而是 frontier -> viewpoint 的过渡：
    - 对每个新 frontier 采样 viewpoint
    - 有 viewpoint 的，放进 active frontiers_
    - 一个 viewpoint 都找不到的，放进 dormant_frontiers_

    同时，这里还会设置 first_new_ftr_，
    让后面的 updateFrontierCostMatrix() 知道 old/new 的分界点在哪里。
  */
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // 对每个本轮新发现的 frontier，尝试围绕它采样可用 viewpoint。
  for (auto& tmp_ftr : tmp_frontiers_) {
    // 这一步会把结果写进 tmp_ftr.viewpoints_。
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // viewpoint 按可见 frontier cell 数量排序，最优 viewpoint 始终放在 front()。
      sort(
          inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
      // 只在第一次插入成功时记录边界，后面这个 iterator 就是“新 frontier 段”的起点。
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
    } else {
      // 没有可用 viewpoint 的 frontier 暂时不作为访问目标，但保留到 dormant 列表里供后续监测。
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }

  // frontier id_ 每轮都按当前 active frontiers_ 顺序重新编号。
  // 这很重要，因为后面的 cost matrix、TSP 节点映射、可视化都默认依赖这套顺序编号。
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
    std::cout << ft.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
            << std::endl;
}

void FrontierFinder::getTopViewpointsInfo(
    const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws,
    vector<Eigen::Vector3d>& averages) {
  /*
    为每个 active frontier 输出一个“粗层级代表 viewpoint”。

    这一步服务的是上层全局 tour 排序，不是最终局部 refinement。
    策略也很简单：
    - 尽量取“离当前位姿不太近”的最佳 viewpoint
    - 如果所有 viewpoint 都很近，那也至少取 coverage 最好的那个
  */
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // view 列表已按 visib_num_ 降序排好，因此这里取到的第一个合法项就是当前最优粗 viewpoint。
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      // 如果所有 viewpoint 都离当前位姿很近，就放宽距离约束，退化为直接取 coverage 最好的那个。
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

void FrontierFinder::getViewpointsInfo(
    const Vector3d& cur_pos, const vector<int>& ids, const int& view_num, const double& max_decay,
    vector<vector<Eigen::Vector3d>>& points, vector<vector<double>>& yaws) {
  /*
    给上层局部 refinement 提供“每个 frontier 的多个候选 viewpoint”。

    输出是分组结构：
    - points[k] / yaws[k]
      对应第 k 个请求 frontier 的候选 viewpoint 集合

    过滤策略：
    - 最多保留 view_num 个
    - 可见性必须高于 best_visib_num * max_decay
    - 优先跳过离当前位姿太近的 viewpoint
  */
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // frontiers_ 是 list，这里按 id 顺序线性扫描找到目标 frontier。
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // 取该 frontier 的 top-N 候选 viewpoint。
        vector<Eigen::Vector3d> pts;
        vector<double> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          // 如果被距离约束全过滤掉了，就再次放宽距离限制，避免某个 frontier 完全拿不到候选 viewpoint。
          for (auto view : frontier.viewpoints_) {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
        yaws.push_back(ys);
      }
    }
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  // 导出当前 active frontier 的原始 voxel 集合，主要供可视化和调试观察。
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  // 导出当前 dormant frontier 的 voxel 集合。
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  // 以 (center, scale) 形式导出包围盒，更适合 RViz marker 直接消费。
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  /*
    按给定 frontier 访问顺序，把整条粗 tour 恢复成一条几何折线。

    注意这不是最终动力学轨迹，只是：
    - 当前位姿 -> 第一个 frontier top viewpoint
    - 第 i 个 frontier -> 第 i+1 个 frontier 的缓存几何路径

    主要用途是给上层做 RViz 可视化。
  */
  // 把 list 先转成“按 id 顺序索引”的迭代器数组，便于后面用 frontier id 直接访问。
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  // 第一段路径没有缓存，需要现场从当前位姿连到第一个 frontier 的 top viewpoint。
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // 后续各段直接从 frontier 内部缓存的 paths_ 里取。
  for (int i = 0; i < frontier_ids.size() - 1; ++i) {
    // paths_ 的第 j 项表示“到 frontier j 的缓存路径”，这里通过 next frontier 的 id 找到对应列。
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}

void FrontierFinder::getFullCostMatrix(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    Eigen::MatrixXd& mat) {
  if (false) {
    // 旧的 symmetric TSP 建模分支，当前主流程不用，保留作对照。
    int dim = frontiers_.size() + 2;
    mat.resize(dim, dim);  // current pose (0), sites, and virtual depot finally

    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_)
        mat(i, j++) = cs;
      ++i;
      j = 1;
    }

    // 当前状态到各 frontier 的代价。
    for (auto ftr : frontiers_) {
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(0, j) = mat(j, 0) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      ++j;
    }
    // 从虚拟 depot 到各 frontier 的代价，统一设成大值。
    for (j = 1; j < dim - 1; ++j) {
      mat(dim - 1, j) = mat(j, dim - 1) = 100;
    }
    // 设一个极小值，确保求解器在这个 formulation 下能闭环。
    mat(0, dim - 1) = mat(dim - 1, 0) = -10000;

  } else {
    /*
      当前实际使用的是 ATSP 建模。

      维度 = frontier 数 + 1：
      - 第 0 行/列       : 当前无人机状态
      - 第 1..N 行/列    : N 个 active frontiers

      其中：
      - 行 1..N、列 1..N 的子块来自前面增量维护好的 frontier 间 cost 缓存
      - 第 0 行则是“当前状态 -> 各 frontier top viewpoint”的在线代价
    */
    int dimen = frontiers_.size();
    mat.resize(dimen + 1, dimen + 1);
    // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
    // 先填 frontier -> frontier 的缓存代价块。
    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_) {
        // std::cout << "(" << i << ", " << j << ")"
        // << ", ";
        mat(i, j++) = cs;
      }
      ++i;
      j = 1;
    }
    // std::cout << "" << std::endl;

    // 再填“当前状态 -> 各 frontier”的起始行。
    // 第 0 列会被置零，因为当前建模里不会真正用“frontier -> current state”这部分。
    mat.leftCols<1>().setZero();
    for (auto ftr : frontiers_) {
      // std::cout << "(0, " << j << ")"
      // << ", ";
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(0, j++) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
    }
    // std::cout << "" << std::endl;
  }
}

void FrontierFinder::findViewpoints(
    const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps) {
  /*
    旧版 viewpoint 搜索函数。

    它的思路是：
    - 固定 sample 位置
    - 通过多条射线扫描不同 yaw slice 的 information gain
    - 再从中选若干 gain 最大的 yaw

    当前主线更常用 sampleViewpoints()：
    - 直接在圆环上采样位置
    - 用 cluster 平均朝向估计 yaw
    - 再统计可见 frontier cell 数

    因此这个函数更像保留的备选实现。
  */
  if (!edt_env_->sdf_map_->isInBox(sample) ||
      edt_env_->sdf_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
    return;

  double left_angle_, right_angle_, vertical_angle_, ray_length_;

  // 先用“sample 指向 frontier 平均位置”的方向作为中心 yaw。
  auto dir = ftr_avg - sample;
  double hc = atan2(dir[1], dir[0]);

  vector<int> slice_gains;
  // 先评估多个水平 slice 的 gain，每个 slice 内再积累若干俯仰方向射线。
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // gain 表示从这个 slice 看出去，能扫到多少 unknown 体素。
    int gain = 0;
    for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3) {
      // 构造这一条视线射线的末端点。
      Vector3d end;
      end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
      end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
      end[2] = sample[2] + ray_length_ * sin(phi_v);

      // 沿这条射线做 raycast，直到遇到障碍或出界为止。
      Vector3i idx;
      raycaster_->input(sample, end);
      while (raycaster_->nextId(idx)) {
        // 视线一旦打到障碍或越界，就停止累积。
        if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
          break;
        // 把射线上命中的 unknown voxel 计作 information gain。
        if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) ++gain;
      }
    }
    slice_gains.push_back(gain);
  }

  // 再把多个 slice gain 按相机水平 FoV 组合起来，得到不同 yaw 方向的总 gain。
  vector<pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(yaw, gain));
  }

  // 选出若干 gain 最高的 yaw，作为该位置下的候选 viewpoint 朝向。
  vps.clear();
  sort(
      yaw_gains.begin(), yaw_gains.end(),
      [](const pair<double, int>& p1, const pair<double, int>& p2) {
        return p1.second > p2.second;
      });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_visib_num_) break;
    Viewpoint vp = { sample, yaw_gains[i].first, yaw_gains[i].second };
    while (vp.yaw_ < -M_PI)
      vp.yaw_ += 2 * M_PI;
    while (vp.yaw_ > M_PI)
      vp.yaw_ -= 2 * M_PI;
    vps.push_back(vp);
  }
}

void FrontierFinder::sampleViewpoints(Frontier& frontier) {
  /*
    当前主线使用的 viewpoint 采样函数。

    过程可以概括成：
    1. 围绕 frontier 平均位置，在多个半径、多个角度上采样 candidate 位置
    2. 过滤掉越界、碰撞、太靠近 unknown 的位置
    3. 估计该位置朝向 frontier 的平均 yaw
    4. 统计这个 pose 能看到多少 frontier cells
    5. 可见数量超过阈值的，记为可用 viewpoint

    最终结果写进 frontier.viewpoints_。
  */
  // 在以 frontier.average_ 为中心的若干同心圆上采样位置。
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      // 候选 viewpoint 必须在有效 box 内，且不落在膨胀障碍内，也不能贴着 unknown 过近。
      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
        continue;

      /*
        用所有 filtered frontier cells 相对 sample_pos 的平均方向来估计一个 yaw。
        这不是严格最优 yaw，但代价低、很稳定，适合作为第一层 viewpoint 采样。
      */
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);

      // 统计这个 pose 下真正“既在 FOV 内又无遮挡”的 frontier cell 数量。
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
      }
      // }
    }
}

bool FrontierFinder::isFrontierCovered() {
  /*
    判断最近一轮地图更新是否已经覆盖掉了当前某个 frontier 的相当一部分。

    这个接口主要给上层 FSM 用来触发重规划。
    逻辑不是看“某个 frontier 是否完全消失”，而是看：
    - 若 update box 和某个 frontier 有重叠
    - 且该 frontier 中已有足够比例的 cells 不再是 frontier
    就认为当前执行中的探索目标已经被覆盖得差不多了，值得重规划。
  */
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      // 不是要求全部消失，只要消失比例达到阈值，就认为这个 frontier 已经“被完成得足够多”。
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  /*
    判断某个候选 viewpoint 是否离 unknown 边界太近。

    这里的目的不是 frontier 检测，而是 viewpoint 安全性：
    如果机体站位太贴近 unknown，说明周围环境仍不够确定，
    不适合把它作为稳定的观测位姿。
  */
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}

int FrontierFinder::countVisibleCells(
    const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster) {
  /*
    统计给定 pose 能看到多少 frontier cells。

    可见的定义要同时满足两层条件：
    1. 几何上在相机 FOV 内
    2. 从 viewpoint 到该 frontier cell 的视线没有被 occupied / unknown 遮挡
  */
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // 先做快速 FOV 过滤，掉在视锥外的点无需再做 raycast。
    if (!percep_utils_->insideFOV(cell)) continue;

    // 再沿 cell -> viewpoint 做射线检查，看中间是否被障碍或 unknown 挡住。
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 ||
          edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib) visib_num += 1;
  }
  return visib_num;
}

void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  /*
    对 frontier cell 点集做体素降采样。

    这一步的主要目的不是为了好看，而是降计算量：
    - viewpoint yaw 估计
    - visible cell 计数
    都不需要在完整 frontier 点集上做。
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  // leaf size 按地图分辨率的倍数设置，因此 down_sample_ 本质上是一个“采样稀疏倍数”。
  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  // 把 yaw 归一化到 [-pi, pi]，避免角度平均和排序时出现跳变。
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt) {
  // 旧工具函数：尝试从某个点附近搜索最近的 free voxel。
  // 当前实现里 BFS 扩展代码已经被注释，基本处于未使用状态。
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);
  // visited_flag_[toadr(pt)] = 1;

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }

    nbrs = sixNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      // if (visited_flag_[adr] == 0)
      // {
      //   init_que.push(nbr);
      //   visited_flag_[adr] = 1;
      // }
    }
  }
  return start_idx;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  // 六邻域：只沿坐标轴正负方向扩一格。
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  // 十邻域：XY 平面八邻域，再补上下两个方向。
  // 当前主流程基本未直接使用。
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  // 二十六邻域：三维体素块里除中心外的全部邻居。
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // frontier voxel 的局部判定条件之一：六邻域里至少有一个 unknown。
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN) return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  // 统一封装 world-grid 地址映射，便于 frontier_flag_ 这种线性数组访问。
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  // 当前体素已经被地图确认是 free。
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  // 判断 voxel 索引是否落在整张地图数组边界内。
  return edt_env_->sdf_map_->isInMap(idx);
}

}  // namespace fast_planner
