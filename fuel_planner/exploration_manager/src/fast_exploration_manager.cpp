// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
/*
  FastExplorationManager 是单机探索里的“算法调度层”。

  和 FastExplorationFSM 的分工可以这样理解：
  - FSM 决定：什么时候开始规划、什么时候重规划、什么时候发布轨迹
  - Manager 决定：下一步去看哪个 frontier、从哪个 viewpoint 看、如何生成到该 viewpoint 的轨迹

  这份文件基本对应论文里的两层核心规划：
  1. frontier -> viewpoint -> global tour
  2. local refinement -> position trajectory -> yaw trajectory

  因此读这个文件时，最重要的是抓住 planExploreMotion() 这一条主链：
  当前状态 -> frontier 更新 -> viewpoint 选择 -> tour 排序 -> 轨迹生成。
*/
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
  /*
    这里故意保留一个显式定义的空构造函数。

    当前没有成员需要在构造时手动初始化，真正的初始化都放在 initialize(nh) 里完成。
    这样写的好处是：
    - 类的生命周期函数都集中放在 .cpp 里，阅读时更集中
    - 将来如果要加初始化逻辑，不用再改头文件接口
  */
}

FastExplorationManager::~FastExplorationManager() {
  /*
    ViewNode 内部有几份跨函数共享的静态资源：
    - A* 搜索器
    - RayCaster
    - 地图指针

    manager 销毁时在这里统一释放，避免残留旧状态。
  */
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  /*
    初始化本 manager 依赖的三类核心模块：
    1. planner_manager_   : 负责位置 / yaw 轨迹生成
    2. frontier_finder_   : 负责 frontier 增量维护和 viewpoint 生成
    3. ViewNode 静态资源 : 负责局部 refinement 中的路径代价评估
  */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);

  /*
    planner_manager_ 初始化后，会顺带把地图环境 EDTEnvironment 也建好。
    这里把它取出来缓存成成员，是因为后面的多个函数都会直接用到：
    - sdf_map_           : 做占据查询、边界查询、raycast 相关逻辑
    - edt_environment_   : 给 frontier / view 评估提供地图环境
  */
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  // FrontierFinder 是 frontier 增量维护与 viewpoint 生成的核心模块。
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  /*
    ed_ / ep_ 这两个结构很容易混：
    - ed_ : ExplorationData，运行期数据缓存，更多是“本轮规划得到的结果”
    - ep_ : ExplorationParam，参数配置，更多是“算法怎么做”
  */
  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  /*
    exploration 层参数，决定“frontier 已经找到了之后，接下来如何挑 viewpoint、如何排序”。

    每个参数的运行时作用如下：
    - refine_local_
      是否启用局部 viewpoint refinement。
      true  : 先做全局 frontier 排序，再对前几个 frontier 的多个候选 viewpoint 做图搜索细化。
      false : 直接使用每个 frontier 的主 viewpoint，不做第二层 refinement。

    - refined_num_
      从全局 tour 的前面最多拿多少个 frontier 进入局部 refinement。
      它控制的是“局部图搜索的层数上限”，不是总 frontier 数。

    - refined_radius_
      进一步限制 refinement 范围的半径阈值。
      当前实现里：如果已经纳入了至少两个 frontier，并且后续 frontier 离当前位姿太远，
      就提前停止，不再把更远的 frontier 加入局部 refinement。

    - top_view_num_
      每个 frontier 最多取多少个候选 viewpoint 进入局部图搜索。
      值越大，局部搜索更充分，但图节点数和边数也会变大。

    - max_decay_
      候选 viewpoint 的“可见性衰减阈值”。
      FrontierFinder 会以最佳 viewpoint 的 visib_num 为基准，只保留高于
      best_visib_num * max_decay 的候选。

    - tsp_dir_
      LKH 求解全局 tour 时使用的工作目录。
      这里会写入 `single.tsp`、`single.par`，并从 `single.txt` 读取结果。

    - relax_time_
      yaw 规划末段的放松时长。
      传给 `planYawExplore()` 后，会让轨迹尾部一小段减弱“必须严格 look-forward”的约束，
      避免快结束时 yaw 还被过强地往前方拉。
  */
  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  /*
    ViewNode 的静态参数用于 viewpoint 之间的代价评估：
    - vm
      把“路径长度”换成“时间代价”时使用的速度尺度，computeCost() 里是 path_length / vm。

    - am
      预留给平移动力学代价的加速度尺度。当前主线公式里相关项被注释掉了，
      所以在这份单机探索主流程里基本不直接起作用。

    - yd
      把 yaw 角度差换成“时间代价”时使用的角速度尺度，computeCost() 里是 yaw_diff / yd。
      后面 `time_lb` 也会直接用它估计“转头至少需要多久”。

    - ydd
      预留给 yaw 加速度代价的尺度。和 am 类似，当前主线里基本未实际参与总代价计算。

    - w_dir
      速度方向惩罚权重。
      若当前速度方向和“去下一个 viewpoint 的前进方向”夹角很大，就会额外加罚，
      避免在局部 refinement 中频繁挑出需要急转的 viewpoint。
  */
  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  /*
    局部 refinement 里每个 viewpoint 候选会变成一个 ViewNode。
    ViewNode::computeCost() 在很多节点之间重复调用，所以把下面这些资源设计成静态共享：
    - astar_ : 如果两点无法直连，用它估路径长度
    - map_   : 查询占据状态 / 是否 unknown / 是否在 box 内
    - caster_: 先做更快的直线穿体素检查
  */
  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  /*
    下面初始化 ViewNode 共享的 RayCaster。

    先解释这几个量各是什么：

    1. resolution_
       地图分辨率，也就是“一个体素边长多少米”。
       例如 resolution_=0.1 时，表示一个 voxel 是 10cm 见方。

    2. origin
       地图原点，也就是这张体素地图在 world 坐标系里的起始偏移。
       后面 RayCaster 需要知道“world 坐标怎么换成体素索引”，所以必须知道 origin。

    3. size
       地图整体尺寸（x/y/z 各方向总共有多大，单位是米）。
       这段代码里虽然把它取出来了，但当前初始化 RayCaster 实际只直接用到了 origin。
       size 更像是把 map region 一起拿出来，便于后续扩展或调试；这一段当前没直接消费它。

    为什么这里要配一个 RayCaster？
    因为后面有很多地方都需要做“从点 A 到点 B 沿直线穿过哪些体素”的判断，比如：
    - shortenPath() 里判断某个中间点能不能删
    - ViewNode::searchPath() 里先尝试直线连接两个 viewpoint

    而 RayCaster::setParams(resolution, origin) 的本质作用就是：
    告诉它“world 坐标 -> voxel 网格索引”该怎么换算。
  */
  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);

  // 把地图的体素分辨率和地图原点交给 RayCaster，之后它才能正确地沿射线枚举经过的体素索引。
  ViewNode::caster_->setParams(resolution_, origin);

  /*
    这里设置 planner_manager_ 里面那套几何路径 A* 的搜索参数。

    注意有两套 A*：
    1. planner_manager_->path_finder_
       用于从当前位姿到 next viewpoint 找一条几何路径
    2. ViewNode::astar_
       用于 viewpoint refinement 时估算两个 viewpoint 之间的路径代价

    这里改的是第 1 套，不是 ViewNode 那套。
  */
  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  /*
    初始化供 LKH 使用的参数文件。

    findGlobalTour() 后续会把代价矩阵写成 single.tsp，
    再通过这里生成的 single.par 调 LKH 求解全局 ATSP tour。
  */
  ofstream par_file(ep_->tsp_dir_ + "/single.par");

  // single.par 是给 LKH 的参数文件，不是 FUEL 自己解析的配置。
  // 它告诉 LKH：
  // - 问题文件在哪
  // - 结果文件写到哪
  // - 跑多少次
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  /*
    单机探索的主入口。

    输入：
    - pos / vel / acc : 当前出发状态
    - yaw             : 当前 yaw 状态，yaw[0] 是角度，后两项一般是 yaw 速度和加速度

    输出：
    - SUCCEED     : planner_manager_ 内部已经准备好了新的位置 / yaw 轨迹
    - NO_FRONTIER : 当前没有可覆盖 frontier，探索结束
    - FAIL        : 有 frontier，但这次没成功生成到下一个 viewpoint 的轨迹

    整体流程分成 5 步：
    1. 更新 frontier
    2. 生成每个 frontier 的主 viewpoint
    3. 全局排序 + 局部 refinement，确定 next viewpoint
    4. 生成到 next viewpoint 的位置轨迹
    5. 在位置轨迹基础上生成 yaw 轨迹
  */
  ros::Time t1 = ros::Time::now();
  // t2 用来统计整条 planExploreMotion() 的总耗时。
  auto t2 = t1;

  /*
    ed_ 是 manager 的“本轮探索中间结果缓存区”。
    每次重新规划前，需要先清空这轮一定会被重建的那几份数据，避免 RViz 里继续显示旧结果。
    这里先清掉：
    - views_       : 粗 viewpoint 的朝向箭头端点
    - global_tour_ : 全局 tour 的可视化连线
  */
  ed_->views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Step 1: 搜索 frontier，并按 cluster 组织。
  frontier_finder_->searchFrontiers();

  // frontier_time / view_time / local_time / tsp_time 这些变量都只是性能统计，不参与算法决策。
  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  /*
    Step 2: 为每个 frontier cluster 生成 viewpoint 候选，并提取当前用于全局排序的主 viewpoint。
    这里会顺便把 frontier_finder_ 内部的数据拷到 ed_ 中，供可视化和后续规划统一使用。
  */
  frontier_finder_->computeFrontiersToVisit();
  /*
    下面几份数据都是 FrontierFinder 内部状态的“只读快照”，拷到 ed_ 里是为了：
    - 规划主流程后续统一访问
    - FSM / 可视化模块统一取数

    它们各自代表：
    - frontiers_
      当前仍然 active、且可继续作为探索目标的 frontier cluster，每个 cluster 是一组 frontier voxel。

    - frontier_boxes_
      每个 active frontier 的包围盒（中心, 尺寸），主要给 RViz 画 marker 和快速观察 cluster 尺度。

    - dead_frontiers_
      当前被放入 dormant 列表的 frontier。
      这些 frontier 不是“从未存在”，而是当前不再作为活跃探索目标参与访问排序。
  */
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  // dormant_frontiers_ 是暂时不可继续扩展 / 不再活跃的 frontier，这里主要拿来做可视化观察。
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

  if (ed_->frontiers_.empty()) {
    // 没有可覆盖 frontier，说明探索已经基本完成。
    ROS_WARN("No coverable frontier.");
    return NO_FRONTIER;
  }

  /*
    对每个 frontier cluster 先抽一个“粗 viewpoint”，供全局排序使用。

    这里会得到三份一一对应的数据：
    - points_[i]
      第 i 个 frontier 当前用于全局排序的代表 viewpoint 位置
    - yaws_[i]
      这个代表 viewpoint 对应的朝向
    - averages_[i]
      第 i 个 frontier cluster 的几何平均位置，只是辅助理解 / 可视化，
      不是无人机真正要飞去的点
  */
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  // ed_->views_ 不是新的目标点，而是给 RViz 画“viewpoint 朝向箭头”的辅助终点。
  // 它和 points_ 配对使用：从 points_[i] 连到 views_[i]，就能在图上看出第 i 个 viewpoint 面朝哪边。
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  double view_time = (ros::Time::now() - t1).toSec();
  ROS_WARN(
      "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
      ed_->points_.size(), view_time);

  /*
    Step 3: 先决定“下一眼看哪里”。

    注意这里不会一次性把所有 frontier 都变成完整飞行轨迹，而是：
    - 全局层先给 frontier 排序
    - 局部层只细化最近几个 frontier 的 viewpoint
    - 最后只取第一个 refined viewpoint 作为当前执行目标
  */
  Vector3d next_pos;
  double next_yaw;
  if (ed_->points_.size() > 1) {
    // 多于一个 frontier 时，先做一次全局 ATSP tour。
    vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
      /*
        对全局 tour 里最靠前的几个 frontier 再做局部 refinement。
        这里只细化最近的若干 frontier，而不是全部细化，以控制计算量。
      */
      t1 = ros::Time::now();

      /*
        refined_ids_
        保存“这次准备做局部 refinement 的 frontier 标识集合”。
        后面 `getViewpointsInfo()` 会拿这些标识，去每个 frontier 里提取 top-N 个候选 viewpoint。

        unrefined_points_
        则是这些 frontier 在粗层级下的代表 viewpoint，仅用于可视化对比：
        方便看 refinement 前后，最终选择的 viewpoint 到底偏了多少。
      */
      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      // knum 是“最多准备细化多少个 frontier”，后面还会被 refined_radius_ 再截一次。
      int knum = min(int(indices.size()), ep_->refined_num_);
      for (int i = 0; i < knum; ++i) {
        auto tmp = ed_->points_[indices[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(indices[i]);
        // 一旦已经选了至少两个 frontier，且当前 frontier 已经比较远，就停止继续扩大 refinement 范围。
        if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
      }

      /*
        为这些 frontier 各取 top-N 个 viewpoint 候选，交给后面的图搜索选择。

        n_points_ 的结构是：
        - 外层下标 k : 第 k 个待 refinement 的 frontier
        - 内层下标 m : 这个 frontier 的第 m 个候选 viewpoint 位置

        n_yaws 与 n_points_ 完全对齐，只是存 yaw。
      */
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      vector<double> refined_yaws;

      // 在候选 viewpoint 图上搜索一条局部代价最小的序列。
      refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);

      /*
        当前轮只执行 refined 序列里的第一个 viewpoint。
        后面的 viewpoint 只是“如果不重新规划，理论上下一批更值得去看的点”，
        真正执行时通常会在飞行过程中再次更新 frontier 并重新规划。
      */
      next_pos = ed_->refined_points_[0];
      next_yaw = refined_yaws[0];

      /*
        下面几段主要把 refined 结果整理成 RViz 可视化用的数据：
        - refined_points_
          refinement 选出的 viewpoint 序列本体
        - refined_views_
          每个 refined viewpoint 的朝向箭头端点
        - refined_views1_ / refined_views2_
          FoV 线段的两端点集合，RViz 里通常用 line list 把它们成对连起来显示视场边界
      */
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        Vector3d view =
            ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
        ed_->refined_views_.push_back(view);
      }
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        vector<Vector3d> v1, v2;
        frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
        frontier_finder_->percep_utils_->getFOV(v1, v2);
        ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
        ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
      }
      double local_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("Local refine time: %lf", local_time);

    } else {
      // 不做 refinement 时，直接取全局 tour 的第一个 viewpoint。
      next_pos = ed_->points_[indices[0]];
      next_yaw = ed_->yaws_[indices[0]];
    }
  } else if (ed_->points_.size() == 1) {
    // 只有一个 frontier 时，就不需要跑全局 TSP，直接对这个 frontier 做局部选择。
    // 这里只有一个 frontier，也仍然刷新一次 frontier 代价缓存，保持 FrontierFinder 内部路径/代价缓存一致。
    frontier_finder_->updateFrontierCostMatrix();
    // global_tour_ 这里直接手工拼成“当前位置 -> 唯一 frontier 主 viewpoint”的折线，供 RViz 画全局 tour。
    ed_->global_tour_ = { pos, ed_->points_[0] };
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

    if (ep_->refine_local_) {
      // 取该 frontier 的 top-N viewpoints，逐个算代价，选总代价最小的那个。
      // 这一分支没有必要再建多层图，因为只剩一个 frontier，本质就是在若干 candidate 中做 argmin。
      ed_->refined_ids_ = { 0 };
      ed_->unrefined_points_ = { ed_->points_[0] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      double min_cost = 100000;
      int min_cost_id = -1;
      // computeCost() 会顺便输出一条几何路径到 tmp_path，但这里我们只真正关心总代价。
      // 代价由“位置变化时间代价 + 速度方向惩罚”和“yaw 变化时间代价”共同决定，最后取两者较大者。
      vector<Vector3d> tmp_path;
      for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
        auto tmp_cost = ViewNode::computeCost(
            pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
        if (tmp_cost < min_cost) {
          min_cost = tmp_cost;
          min_cost_id = i;
        }
      }
      next_pos = ed_->n_points_[0][min_cost_id];
      next_yaw = n_yaws[0][min_cost_id];
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      // 不做 refinement 时，直接使用该 frontier 的主 viewpoint。
      next_pos = ed_->points_[0];
      next_yaw = ed_->yaws_[0];
    }
  } else
    ROS_ERROR("Empty destination.");

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

  /*
    Step 4: 为选中的 next viewpoint 生成位置轨迹。

    这里先根据 yaw 差值估计一个“最短允许时间” time_lb，
    再把这个下界传给位置轨迹生成器，避免位置轨迹过短而来不及完成转头。
  */
  t1 = ros::Time::now();

  // 根据当前 yaw 与目标 yaw 的差值估计转向所需的最小时间。
  double diff = fabs(next_yaw - yaw[0]);
  /*
    这里只做一个简单但很实用的下界估计：
    位置轨迹至少要长到足够完成这次转头。

    注意这个 `time_lb` 不是“整条轨迹的最终时长”，而是位置轨迹生成器必须满足的最低时长约束。
    真正轨迹总时长还会受路径长度、速度/加速度限制、B-spline 优化结果共同影响。
  */
  double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

  /*
    先用几何 A* 找到当前位置到 next viewpoint 的无碰撞路径。
    位置轨迹真正的 B-spline 生成会在 planner_manager_ 里完成。
  */
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
    ROS_ERROR("No path to next viewpoint");
    return FAIL;
  }
  // 注意：这里得到的还是几何路径，不是最终执行的 B-spline。
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();

  // A* 输出路径点通常较密，先压缩到“保关键拐点”的版本，减轻后续轨迹优化负担。
  shortenPath(ed_->path_next_goal_);

  /*
    按到目标的路径长度分三种情况生成位置轨迹：
    1. 很近   : 直接做 waypoint-based 轨迹优化
    2. 很远   : 只截取前半段当中间目标，先飞到阶段目标
    3. 中距离 : 直接做 kinodynamic search + B-spline 优化，精确到达 viewpoint
  */
  const double radius_far = 5.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close) {
    // 很近：没必要跑完整 kinodynamic 搜索，直接做 waypoint-based 优化即可。
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;

  } else if (len > radius_far) {
    /*
      很远：只取前 radius_far 米左右的路径作为中间目标。
      这样做更像 receding-horizon，也更适合处理死胡同或长距离绕障。
    */
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    // 这里的 next_goal_ 是阶段目标，不一定就是原始 frontier 的最终 viewpoint。
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
    // if (!planner_manager_->kinodynamicReplan(
    //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
    //   return FAIL;
    // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  } else {
    // 中距离：直接做 kinodynamic search + B-spline 优化，精确到 next viewpoint。
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
  }

  // 防御性检查：位置轨迹总时长不应该短于 yaw 变化的理论下界。
  // 否则就会出现“位置先到，但 yaw 理论上来不及转过去”的不一致。
  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
    ROS_ERROR("Lower bound not satified!");

  // Step 5: 在位置轨迹固定后，再生成一条 yaw look-forward 轨迹。
  // 这个顺序很重要，因为 yaw 规划会参考位置轨迹未来一小段的前进方向。
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

  double traj_plan_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // 这里当前写法下 yaw_time 基本接近 0，因为 t1 在上一行刚被刷新。
  // 更像是一个遗留统计位，不影响主流程。
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  double total = (ros::Time::now() - t2).toSec();
  ROS_WARN("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  /*
    压缩几何路径，只保留真正必要的中间点。

    这不是简单的按距离抽稀：
    - 如果某个点离上一个保留点足够远，直接保留
    - 否则再检查“上一个保留点 -> 下下个点”能否直连
      - 能直连：当前中间点可删
      - 不能直连：说明它是绕障所必需的关键点，必须保留
  */
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }

  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    // 如果当前点离上一个保留点已经足够远，先直接保留，避免过度压缩。
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // 距离上看似可删，但仍需保证直连不会穿进 occupied / unknown 区域。
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        // 只要直连过程中碰到膨胀障碍或未知区域，说明这个中间点是绕障所必需的，不能删。
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // 后续一些轨迹参数化过程更偏好至少 3 个点；若只有首尾点，则补一个中点。
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
  /*
    对所有 frontier 的主 viewpoint 做一次全局粗排序。

    输入：
    - 当前状态 cur_pos / cur_vel / cur_yaw

    输出：
    - indices : frontier 的访问顺序（按 frontier 索引表示）

    做法：
    1. 让 FrontierFinder 构建完整代价矩阵
    2. 写成 TSPLIB/ATSP 文件
    3. 调用 LKH 求解最优 tour
    4. 再把 solver 输出解析回 frontier 下标
  */
  auto t1 = ros::Time::now();

  /*
    当前状态到各 frontier、以及 frontier 两两之间的代价都在这个矩阵里。

    对当前使用的 ATSP 建模来说：
    - 第 0 行/列    : 当前无人机状态这个“起点锚点”
    - 第 1..N 行/列 : N 个 active frontier

    其中：
    - cost_mat(0, j) : 当前状态 -> 第 j-1 个 frontier 的代价
    - cost_mat(i, j) : 第 i-1 个 frontier -> 第 j-1 个 frontier 的代价
    - cost_mat(i, 0) : 作为 open-tour 锚点辅助项，当前实现里会被置 0
  */
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);

  // 维度 = frontier 数 + 1，其中额外那 1 个就是“当前状态”这个起点锚点。
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // 把代价矩阵写成 TSPLIB 文件，后面交给 LKH 求解。
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  /*
    LKH 这里读取的是整数边权，因此把 double cost 放大后取整写入。
    scale = 100 表示保留到 0.01 量级的代价分辨率。
    如果不放大，很多小数代价在取整后会丢失区分度。
  */
  const int scale = 100;
  if (false) {
    // 这一支是遗留的对称 TSP 写法，当前主流程不会进入。
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    /*
      Use Asymmetric TSP：从 i 到 j 的代价不一定等于从 j 到 i 的代价。
      这在 FUEL 里更合理，因为：
      - 当前状态到 frontier 的代价和 frontier 之间的代价本来就不是同一种条件
      - viewpoint 代价里含有 yaw 和速度方向项，天然是有方向性的
    */
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // 调用外部 LKH 求解器。
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // 读取 LKH 输出的最优 tour。
  // LKH 输出的节点编号是 1-based，不是 C++ 里的 0-based。
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // 这支是旧的 symmetric TSP 解析逻辑，保留在这里主要用于对照。
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    /*
      ATSP 解析逻辑：
      - LKH 输出里的 1 号节点是“当前状态”锚点
      - 2..dimension 对应 frontiers

      因此读回后：
      - 遇到 1  : 跳过，因为它不是 frontier
      - 遇到 -1 : tour 结束
      - 其他 id : 用 id - 2 映射回 frontier 序号
    */
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      // 因为 solver 里的节点编号从 1 开始，且 1 号节点保留给当前状态，所以 frontier 要减 2。
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  /*
    额外恢复成一条可视化路径，方便在 RViz 中观察全局访问顺序。
    这条 `global_tour_` 是“沿全局 frontier 访问顺序拼起来的几何折线”，
    不是最终要执行的动力学可行轨迹。
  */
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) {
  /*
    对“最近几个 frontier”的 viewpoint 候选做局部 refinement。

    输入：
    - n_points / n_yaws :
      每个 frontier 对应一组 viewpoint 候选

    输出：
    - refined_pts / refined_yaws :
      从这些候选中选出的局部最优 viewpoint 序列

    思想：
    - 每个 frontier 对应图中的一层
    - 相邻两层之间全连边
    - 边代价由 ViewNode::computeCost() 估计
    - 最后在图上跑 Dijkstra，得到局部最优的 viewpoint 序列
  */
  // 下面三个时间只是 profiling：
  // - create_time : 建图耗时
  // - search_time : Dijkstra 搜索耗时
  // - parse_time  : 把搜索结果转回几何/可视化数据的耗时
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  /*
    Step 1: 创建 viewpoint 选择图。

    图的结构是一个分层 DAG：
    - 第 0 层        : 当前无人机状态
    - 第 1..K 层     : K 个待 refinement frontier 的候选 viewpoints
    - 边只从前一层指向后一层，不回连

    这样 Dijkstra 的结果就对应：
    “每层选一个 viewpoint，使得从当前状态一路走过去的总代价最小”。
  */
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // 起点是“当前无人机状态”。
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;

  // 这里给起点额外补上当前速度，是为了让 computeCost() 能考虑“当前速度方向”和目标方向的夹角。
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // 逐层加入各 frontier 的 viewpoint 候选。
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // 当前 frontier 的每个 viewpoint 都是一个候选节点。
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // 与上一层全连边，让图搜索自己决定哪种转移最划算。
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      /*
        注意这里按“当前代码行为”理解：
        最后一层只把第一个 candidate 记成 `final_node`，然后直接 `break`。
        也就是说，局部图搜索最终是朝这个固定终点收敛的；
        前面几层的 viewpoint 会被优化选择，但最后一层在当前实现里没有把所有 candidate 都接进去。
      */
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }

    // 当前层构建完后，作为下一层的前驱集合。
    // 调试输出里打印的就是每层实际放进图里的候选节点个数。
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Step 2: 在图上搜索局部最优的 viewpoint 序列。
  // 目标是从当前状态节点 first 走到终点 final_node，得到总代价最小的一条分层路径。
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Step 3: 把图搜索结果转回 point + yaw 序列。
  // path[0] 是起点（当前状态），真正的 refined viewpoints 从 path[1] 开始。
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  /*
    Step 4: 额外拼一条局部 tour 连线出来，主要给 RViz 可视化使用。

    refined_tour_ 的作用是“把局部 refinement 最终选出的几个 viewpoint 用几何路径串起来”，
    这样在 RViz 里能直观看出 refinement 后的访问顺序与绕障关系。
  */
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  // 这里临时把启发式权重调低到 1.0，使 A* 更接近真实代价搜索，
  // 便于为可视化拼出一条更稳定的几何连接路径，而不是过于激进地依赖启发式。
  ViewNode::astar_->lambda_heu_ = 1.0;
  // 可视化连线不需要特别细的分辨率，0.2m 已足够表达整体几何形状。
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }

  // 这里把 ViewNode 共享 A* 的参数调回 launch 里默认的更激进启发式（algorithm.xml 里是 10000），避免影响别处。
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

}  // namespace fast_planner
