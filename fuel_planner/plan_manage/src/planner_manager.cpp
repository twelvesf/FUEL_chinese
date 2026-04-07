// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

namespace fast_planner {
/*
  FastPlannerManager 是“把目标点真正变成可执行轨迹”的那一层。

  如果说：
  - FastExplorationFSM 决定什么时候该规划
  - FastExplorationManager 决定下一步看哪个 frontier / viewpoint

  那么本文件负责的是最后这一步：
  - 先找路径
  - 再把路径参数化成 B-spline
  - 再做位置轨迹优化
  - 最后再生成 yaw 轨迹

  在当前单机 FUEL 主线里，最常走的是这两条接口：
  1. planExploreTraj()
     适用于已有几何路径 / waypoint 序列的情况
  2. kinodynamicReplan()
     适用于需要直接从当前动力学状态精确到达目标位姿的情况

  两条主线最终都会把结果写进 local_data_，然后由上层 FSM 序列化后发给 traj_server。
*/
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {
  /*
    当前构造函数保持为空，真正的初始化都集中在 initPlanModules(nh)。
    这样上层可以先创建 manager，再按 launch 参数决定启用哪些子模块。
  */
}

FastPlannerManager::~FastPlannerManager() {
  // 当前没有手动释放的裸指针资源，这里主要保留生命周期日志。
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /*
    初始化 planner manager 依赖的地图、搜索器、优化器和若干辅助模块。

    这里读入的 `manager/*` 参数基本可以分成 3 组：
    1. 物理约束
       max_vel / max_acc / max_jerk / max_yawdot
    2. 轨迹表示与局部规划尺度
       local_traj_len_ / ctrl_pt_dist / bspline_degree_
    3. 优化目标与安全阈值
       clearance_ / min_time_

    另外几项容易混：
    - accept_vel_ / accept_acc_
      更像“放宽后的验收阈值”，不是硬约束上限
    - dynamic_
      是否按动态环境逻辑处理
  */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
  nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
  nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
  nh.param("manager/min_time", pp_.min_time_, false);

  /*
    下面这些开关决定 manager 会把哪些算法子模块真正实例化出来。
    当前 office 单机探索主线最相关的是：
    - use_geometric_path
    - use_kinodynamic_path
    - use_optimization

    topo / active_perception 这一组更多服务别的 demo 或更通用的规划实验。
  */
  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization,
      use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_active_perception", use_active_perception, false);

  local_data_.traj_id_ = 0;

  /*
    先建地图环境：
    - sdf_map_
      真正存 occupancy / ESDF / inflate occupancy 的地图主体
    - edt_environment_
      给搜索器和优化器提供统一环境接口
  */
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    // 几何 A*：只关心无碰撞路径几何，不显式考虑速度/加速度状态。
    path_finder_.reset(new Astar);
    // path_finder_->setParam(nh);
    // path_finder_->setEnvironment(edt_environment_);
    // path_finder_->init();
    path_finder_->init(nh, edt_environment_);
  }

  if (use_kinodynamic_path) {
    // Kinodynamic A*：显式考虑起点/终点速度与加速度，更适合精确衔接当前飞行状态。
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    /*
      这里预先创建多份 B-spline 优化器。

      当前常见用法约定上是：
      - bspline_optimizers_[0] : 位置轨迹优化
      - bspline_optimizers_[1] : yaw 轨迹优化
      - 其余若干槽位          : topo 重规划时并行优化多条候选轨迹
    */
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    // TopologyPRM 用于“碰撞后找多条拓扑不同的绕障路径”，主要服务 topoReplan()。
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }

  if (use_active_perception) {
    /*
      这一组模块主要用于主动感知 / heading / 可见性相关实验。
      在当前单机 FUEL 主线里，不是每次都会用到，但 manager 预留了统一入口。
    */
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    heading_planner_.reset(new HeadingPlanner(nh));
    heading_planner_->setMap(sdf_map_);
    visib_util_.reset(new VisibilityUtil(nh));
    visib_util_->setEDTEnvironment(edt_environment_);
    plan_data_.view_cons_.idx_ = -1;
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  // 仅缓存全局 waypoint 序列，真正转成轨迹是在 planGlobalTraj() 里完成。
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {
  /*
    沿当前已生成的位置轨迹向前做一个短程碰撞检查。

    检查方式很直接：
    - 从“当前时间对应的位置”开始
    - 每 0.02s 往前采样一个未来点
    - 最多看前方 6 米左右
    - 只要未来点落入 inflate occupancy，就认为轨迹即将碰撞

    输出参数 `distance` 返回的是：
    从当前点沿轨迹往前推进，到首次检测到碰撞点的大致弧长距离。
  */
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    // double dist = edt_environment_->sdf_map_->getDistance(fut_pt);
    if (sdf_map_->getInflateOccupancy(fut_pt) == 1) {
      distance = radius;
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
      std::cout << "collision at: " << fut_pt.transpose() << std::endl;
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d& start_pt,
    const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
    const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double& time_lb) {
  /*
    当前位置轨迹规划主线之一：
    从“当前位置 + 当前速度 + 当前加速度”直接规划到目标状态。

    这条链路适合：
    - 目标点不算特别远
    - 需要精确衔接当前动力学状态
    - 希望终点状态约束明确地被纳入搜索与优化

    流程分 4 步：
    1. kinodynamic A* 搜索一条动力学可行的离散轨迹
    2. 对搜索结果采样，得到 point_set 和边界导数
    3. 参数化成位置 B-spline 控制点
    4. 用 B-spline 优化器做平滑 / 安全 / 可行性优化
  */
  std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
            << ", " << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", "
            << end_vel.transpose() << endl;

  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  auto t1 = ros::Time::now();

  /*
    第一次 search(..., true) 一般表示使用更完整/更严格的终点约束或 shot 逻辑。
    若失败，再退一步用 `false` 做一次更宽松的重试。
  */
  kino_path_finder_->reset();
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
  if (status == KinodynamicAstar::NO_PATH) {
    ROS_ERROR("search 1 fail");
    // Retry
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[Kino replan]: Can't find path." << endl;
      return false;
    }
  }
  // 这里只是为了可视化/调试，把 kino 搜到的离散轨迹先存下来；最终执行的仍然是后面优化后的 B-spline。
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  double t_search = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  /*
    把 kino 搜索结果按固定时间间隔重新采样，准备参数化成 B-spline。

    ts 的直观含义可以理解成：
    相邻 B-spline 控制点大约相隔 `ctrl_pt_dist` 米时，在最大速度量级下对应多大的时间跨度。
  */
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  // std::cout << "point set:" << std::endl;
  // for (auto pt : point_set) std::cout << pt.transpose() << std::endl;
  // std::cout << "derivative:" << std::endl;
  // for (auto dr : start_end_derivatives) std::cout << dr.transpose() << std::endl;

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

  /*
    把离散采样点参数化为一条初始 B-spline，然后交给优化器细化。

    这里常见的 cost_function：
    - NORMAL_PHASE
      平滑 + 障碍距离 + 动力学可行性 + 起终点边界
    - 再加上 MINTIME
      额外鼓励更短时间
  */
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;
  vector<Eigen::Vector3d> start, end;
  init.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  // time_lb 常由上层根据 yaw 转向需求给出，要求位置轨迹不能短于这个最小时长下界。
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

  // 这里打印的是优化后轨迹对边界状态的保持误差，主要用于调试参数化/优化是否稳定。
  vector<Eigen::Vector3d> start2, end2;
  local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
  std::cout << "State error: (" << (start2[0] - start[0]).norm() << ", "
            << (start2[1] - start[1]).norm() << ", " << (start2[2] - start[2]).norm() << ")"
            << std::endl;

  double t_opt = (ros::Time::now() - t1).toSec();
  ROS_WARN("Kino t: %lf, opt: %lf", t_search, t_opt);

  // t1 = ros::Time::now();

  // // Adjust time and refine

  // double dt;
  // for (int i = 0; i < 2; ++i)
  // {
  //   NonUniformBspline pos = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);
  //   pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  //   pos.lengthenTime(min(1.01, pos.checkRatio()));
  //   double duration = pos.getTimeSum();
  //   dt = duration / double(pos.getControlPoint().rows() - pp_.bspline_degree_);

  //   point_set.clear();
  //   for (double time = 0.0; time <= duration + 1e-4; time += dt)
  //     point_set.push_back(pos.evaluateDeBoorT(time));
  //   NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivatives,
  //   pp_.bspline_degree_, ctrl_pts);
  //   bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_function, 1, 1);
  // }
  // local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  // iterative time adjustment

  // double to = pos.getTimeSum();
  // pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  // bool feasible = pos.checkFeasibility(false);

  // int iter_num = 0;
  // while (!feasible && ros::ok()) {

  //   feasible = pos.reallocateTime();

  //   if (++iter_num >= 3) break;
  // }

  // // pos.checkFeasibility(true);
  // // cout << "[Main]: iter num: " << iter_num << endl;

  // double tn = pos.getTimeSum();

  // cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  // if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  // t_adjust = (ros::Time::now() - t1).toSec();

  // // save planned results

  // local_data_.position_traj_ = pos;

  // double t_total = t_search + t_opt + t_adjust;
  // cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ",
  // optimize: " << t_opt
  //      << ", adjust time:" << t_adjust << endl;

  // pp_.time_search_   = t_search;
  // pp_.time_optimize_ = t_opt;
  // pp_.time_adjust_   = t_adjust;

  // int rd = rand() % 2;
  // if (rd == 0) {
  //   updateTrajInfo();
  //   return true;
  // } else
  //   return false;

  updateTrajInfo();
  return true;
}

void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d>& tour,
    const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc, const double& time_lb) {
  /*
    当前位置轨迹规划主线之二：
    已经有一条几何路径 / waypoint 序列时，把它变成可执行的位置 B-spline。

    这条链路通常出现在：
    - 目标比较近
    - 或者上层已经先用几何 A* 截出了一段阶段路径

    它和 kinodynamicReplan() 的区别是：
    - 这里先走“waypoints -> polynomial traj -> B-spline”
    - 不直接在搜索阶段建模完整动力学状态
  */
  if (tour.empty()) ROS_ERROR("Empty path to traj planner");

  // 先把 waypoint 序列整理成矩阵形式，交给多项式轨迹初始化器。
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  // 初始每段时间按“距离 / (0.5 * max_vel)”给一个保守估计，避免初始多项式过激。
  for (int i = 0; i < pt_num - 1; ++i)
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  // 先生成一条连接这些 waypoints 的多项式轨迹，作为 B-spline 的初始化参考。
  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

  /*
    再把这条初始多项式轨迹离散采样，重新参数化成 B-spline。

    seg_num 的作用是控制控制点密度：
    - 和轨迹总长度成正比
    - 至少保留 8 段，避免控制点太少导致优化自由度不够
  */
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double dt = duration / double(seg_num);

  std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt << std::endl;

  // 位置采样点 + 起终点一二阶导数，一起构成 B-spline 参数化所需输入。
  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  // 位置优化阶段仍然复用 bspline_optimizers_[0]。
  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

  vector<Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  // 如果上层给了 yaw 对应的最小时长下界，就在这里一起约束位置轨迹。
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  updateTrajInfo();
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  /*
    这条接口主要服务“全局 waypoint -> 全局参考轨迹 -> 当前局部截段”的规划流程，
    更偏 topo / 全局导航那一支，不是当前单机探索主线最常走的函数。

    它做两件事：
    1. 把 global_waypoints_ 变成一条全局 polynomial trajectory
    2. 再从全局轨迹中截一小段，参数化成当前可执行的局部 B-spline
  */
  plan_data_.clearTopoPaths();

  // 先取出外部设置好的全局 waypoint，并把当前位置塞到最前面作为起点。
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // 如果相邻 waypoint 间距太大，就在线性插值补中间点，避免全局参考轨迹过粗。
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();
    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;
      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }
  inter_points.push_back(points.back());

  // waypointsTraj 至少希望看到 3 个点；若只有起终点，则人工补一个中点。
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // 全局参考轨迹用 polynomial trajectory 表示即可，不急着一开始就上 B-spline。
  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  // 起止段额外补一点加减速时间，让参考轨迹更符合速度从零起步/收尾的直觉。
  time(0) += pp_.max_vel_ / (2 * pp_.max_acc_);
  time(time.rows() - 1) += pp_.max_vel_ / (2 * pp_.max_acc_);

  PolynomialTraj gl_traj;
  PolynomialTraj::waypointsTraj(pos, zero, zero, zero, zero, time, gl_traj);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // 然后从全局轨迹中截一段局部轨迹，作为此刻实际执行的 local B-spline。
  double dt, duration;
  Eigen::MatrixXd ctrl_pts = paramLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, pp_.bspline_degree_, dt);

  std::cout << "ctrl pt: " << ctrl_pts.rows() << std::endl;

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_ = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  /*
    拓扑重规划主线。

    这套逻辑主要用于：
    - 当前局部轨迹与障碍发生冲突
    - 需要找多条拓扑不同的绕障候选
    - 再并行优化，挑一条最好轨迹

    它不是当前 FUEL 单机 frontier 探索 office demo 的主路径，
    但和主线共享同一个 planner manager，因此理解数据流仍然很有帮助。
  */
  ros::Time t1, t2;

  /*
    第一步先从全局参考轨迹中，截取当前位置开始的一段局部轨迹。
    后面无论是否碰撞，都是围绕这段局部 B-spline 做 refine 或 replan。
  */
  ros::Time time_now = ros::Time::now();
  double t_now = (time_now - global_data_.global_start_time_).toSec();
  double local_traj_dt, local_traj_duration;

  Eigen::MatrixXd ctrl_pts = paramLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, pp_.bspline_degree_, local_traj_dt);
  local_data_.start_time_ = time_now;

  std::cout << "dt: " << local_traj_dt << ", dur: " << local_traj_duration << std::endl;

  if (!collide) {
    // 没碰撞时也不是直接原样用，而是再做一次 refine，让局部轨迹更平滑更安全。
    refineTraj(init_traj);
    double time_change = init_traj.getTimeSum() - local_traj_duration;
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(
        local_data_.position_traj_, t_now, local_traj_duration + time_change + t_now, time_change);
    // local_data_.position_traj_ = init_traj;
    // global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
  } else {
    /*
      真正的 topo 重规划分支：
      - 先找出初始局部轨迹的碰撞区间
      - 再在碰撞区间前后找安全连接段
      - 最后让 topo_prm_ 搜多条拓扑不同的几何路径
    */
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      // 这种情况说明局部轨迹尾部已经陷进障碍里，难以截出一个有效“碰撞后再恢复”的区间。
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
    } else {
      // 在碰撞区间前后之间搜索多条拓扑不同的候选路径。
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr> graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
          raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      // 对每条 topo 路径并行做一轮“路径引导 + 正常优化”，得到多条位置 B-spline 候选。
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
            local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;

      // 再从多个候选位置轨迹里选一条最好的，并做最终 refine。
      NonUniformBspline best_traj;
      selectBestTraj(best_traj);
      refineTraj(best_traj);
      double time_change = best_traj.getTimeSum() - local_traj_duration;

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
          local_traj_duration + time_change + t_now, time_change);
    }
  }
  updateTrajInfo();

  double tr = (ros::Time::now() - time_now).toSec();
  ROS_WARN("Replan time: %lf", tr);

  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // 当前策略很朴素：直接按 jerk 排序，取 jerk 最小的那条。
  // 这意味着在二阶段 topo 优化之后，“更平顺”被当作首要选拔标准。
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
      [](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj) {
  /*
    对已经选中的位置 B-spline 再做一次细化优化。

    可以把它理解成：
    - 上一阶段先找到“哪条候选轨迹大方向最好”
    - 这里再用更纯粹的 NORMAL_PHASE 在这条轨迹附近做一轮局部打磨
  */
  ros::Time t1 = ros::Time::now();
  plan_data_.no_visib_traj_ = best_traj;

  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;

  // ViewConstraint view_cons;
  // visib_util_->calcViewConstraint(best_traj, view_cons);
  // plan_data_.view_cons_ = view_cons;
  // if (view_cons.idx_ >= 0)
  // {
  //   cost_function |= BsplineOptimizer::VIEWCONS;
  //   bspline_optimizers_[0]->setViewConstraint(view_cons);
  // }

  // 从当前 best_traj 的控制点重新开一轮优化，但保持相同边界状态。
  Eigen::MatrixXd ctrl_pts = best_traj.getControlPoint();
  double dt = best_traj.getKnotSpan();
  vector<Eigen::Vector3d> start1, end1;
  best_traj.getBoundaryStates(2, 0, start1, end1);

  bspline_optimizers_[0]->setBoundaryStates(start1, end1);
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_function, 2, 2);
  best_traj.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  // 打印 refine 前后边界状态误差，便于观察优化有没有把边界拉坏。
  vector<Eigen::Vector3d> start2, end2;
  best_traj.getBoundaryStates(2, 2, start2, end2);
  for (int i = 0; i < 3; ++i)
    std::cout << "error start: " << (start1[i] - start2[i]).norm() << std::endl;
  for (int i = 0; i < 1; ++i)
    std::cout << "error end  : " << (end1[i] - end2[i]).norm() << std::endl;
}

void FastPlannerManager::updateTrajInfo() {
  /*
    每次位置轨迹或 yaw 轨迹更新后，都需要把 local_data_ 里“供外部消费的派生信息”补齐。

    对位置轨迹而言，这里会统一刷新：
    - velocity_traj_
    - acceleration_traj_
    - start_pos_
    - duration_
    - traj_id_

    其中 `traj_id_` 很关键：
    上层 FSM / traj_server 会用它区分“这是第几条新轨迹”，避免把旧轨迹和新轨迹混在一起。

    注意这里故意不改 `start_time_`：
    轨迹从什么时候开始生效，是更上层调度逻辑（FSM / replan 触发点）决定的，
    所以 start_time_ 由调用方在合适时机单独写入。
  */
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
    Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  /*
    对一条已有 B-spline 做“拉长时间后重新参数化”。

    常见用途是：
    当轨迹速度/加速度偏激时，先整体拉长时间，再重新采样/参数化成新的控制点矩阵。
    当前主流程里这段更多是工具函数和历史实验逻辑的支撑。
  */
  int prev_num = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();

  int seg_num = bspline.getControlPoint().rows() - pp_.bspline_degree_;
  // 当前实现把单次拉长比例限制得很小，更像保守的微调而不是大幅重分配。
  ratio = min(1.01, ratio);

  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  // 重新按新时长均匀采样，然后重新做一次 B-spline 参数化。
  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt)
    point_set.push_back(bspline.evaluateDeBoorT(time));
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, plan_data_.local_start_end_derivative_, pp_.bspline_degree_, ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(
    double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id) {
  /*
    对单条 topo 几何路径做“两阶段 B-spline 优化”。

    这里会在线程里并行运行，所以每条候选路径都用不同的 `traj_id`
    对应不同的 bspline_optimizers_[traj_id] 与 plan_data_ 槽位。
  */
  auto t1 = ros::Time::now();

  // 先根据 guide path 长度决定这一条候选轨迹应有多少段控制点。
  int seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  seg_num = max(6, seg_num);  // Min number required for optimizing
  double dt = duration / double(seg_num);
  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(start_t, duration, dt);

  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);
  vector<Eigen::Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);

  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  /*
    把几何 guide path 离散成与 B-spline 控制点数量相匹配的 guide points。

    不同 B-spline degree 下，可自由优化的内部控制点数量不同，
    所以这里对 3/4/5 阶采用了不同的对齐方式。
  */
  vector<Eigen::Vector3d> tmp_pts, guide_pts;
  if (pp_.bspline_degree_ == 3 || pp_.bspline_degree_ == 5) {
    topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2, tmp_pts);
    guide_pts.insert(guide_pts.end(), tmp_pts.begin() + 2, tmp_pts.end() - 2);
    if (guide_pts.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("Incorrect guide for 3 degree");
  } else if (pp_.bspline_degree_ == 4) {
    topo_prm_->pathToGuidePts(guide_path, int(2 * ctrl_pts.rows()) - 7, tmp_pts);
    for (int i = 0; i < tmp_pts.size(); ++i) {
      if (i % 2 == 1 && i >= 5 && i <= tmp_pts.size() - 6) guide_pts.push_back(tmp_pts[i]);
    }
    if (guide_pts.size() != int(ctrl_pts.rows()) - 8) ROS_WARN("Incorrect guide for 4 degree");
  }

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;

  double tm1 = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // 第一阶段：让轨迹先贴近 guide path，大方向先走对。
  bspline_optimizers_[traj_id]->setBoundaryStates(start, end);
  bspline_optimizers_[traj_id]->setGuidePath(guide_pts);
  bspline_optimizers_[traj_id]->optimize(ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);
  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  double tm2 = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // 第二阶段：在已有大方向基础上，再做平滑 + 安全 + 可行性优化。
  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  // if (pp_.min_time_)
  //   cost_func |= BsplineOptimizer::MINTIME;
  bspline_optimizers_[traj_id]->setBoundaryStates(start, end);
  bspline_optimizers_[traj_id]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  double tm3 = (ros::Time::now() - t1).toSec();
  // ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::paramLocalTraj(double start_t, double& dt, double& duration) {
  /*
    从全局参考轨迹里截取一个“球形局部段”，并直接参数化成 B-spline 控制点矩阵。

    这里的“球形局部段”不是按固定时间长度截，而是：
    从 start_t 出发，沿全局轨迹往前走，直到空间距离超过 `local_traj_len_` 为止。

    这样截出来的局部段更贴近“局部重规划只关心前方一小片空间”的需求。
  */
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;
  global_data_.getTrajInfoInSphere(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
      start_end_derivative, dt, duration);

  // 再把这段局部采样点及其边界导数参数化成 B-spline。
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);
  // 缓存边界导数，供后续 reparamBspline() 等步骤继续复用。
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(
    const double& start_t, const double& duration, const double& dt) {
  /*
    从全局参考轨迹里按“固定时长 + 固定采样间隔”重新截一段局部轨迹并参数化。

    它和 paramLocalTraj() 的区别是：
    - paramLocalTraj() 先按空间半径决定局部段
    - reparamLocalTraj() 直接按给定 duration / dt 重采样

    topo 候选轨迹优化时更常用后者，因为不同 guide path 会希望匹配不同控制点密度。
  */
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajInfoInDuration(start_t, duration, dt, point_set, start_end_derivative);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  // 重新参数化成 B-spline 控制点矩阵。
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
    vector<Eigen::Vector3d>& colli_end, vector<Eigen::Vector3d>& start_pts,
    vector<Eigen::Vector3d>& end_pts) {
  /*
    在当前局部初始轨迹上找出“进入碰撞”和“离开碰撞”的区间。

    这一步的输出有两层意义：
    - colli_start / colli_end
      标记碰撞区间的入口与出口
    - start_pts / end_pts
      提取碰撞区间前后的安全轨迹段采样点，供 topo_prm_ 建连接时使用
  */
  bool last_safe = true, safe;
  double t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  // 先粗采样整段局部轨迹，找到安全->碰撞、碰撞->安全的切换边界。
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {
    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    // 这里不是查 occupancy，而是查 coarse EDT 是否小于 topo 模块要求的安全 clearance。
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /*
    再把碰撞区间前后的安全部分各采样一段出来。
    topo_prm_ 后面会把这些点作为连接起点和终点的候选安全走廊信息。
  */
  double dt = initial_traj->getKnotSpan();
  int sn = ceil((t_s - t_m) / dt);
  dt = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getKnotSpan();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  /*
    通用 yaw 规划接口。

    基本思路是 look-forward：
    - 沿位置轨迹每隔 dt_yaw 取一个时刻
    - 看当前点到 forward_t 秒后位置的方向
    - 把这个方向当成 yaw waypoint
    - 最后再整体拟合成一条平滑 yaw B-spline

    这条接口更通用，而当前单机探索主线更常用后面的 planYawExplore()。
  */
  auto t1 = ros::Time::now();
  // 先从位置轨迹中提取“朝前看”的 yaw waypoints。

  auto& pos = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  // 先给 yaw 一个大致时间分辨率，再按总时长等分成若干段。
  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // waypt_idx 记录的是这些 yaw waypoints 应约束到哪几个内部 B-spline 段上。

  for (int i = 0; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    // pd 表示未来前视方向，yaw 就朝这个方向。
    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // 接着构造 yaw B-spline 的控制点矩阵，并用起终边界状态先固定两头。

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // 再把中间的 yaw waypoints 当作软约束，交给优化器做平滑拟合。
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                  BsplineOptimizer::START | BsplineOptimizer::END;

  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw[0], 0, 0),
    Eigen::Vector3d(start_yaw[1], 0, 0), Eigen::Vector3d(start_yaw[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw[0], 0, 0),
    Eigen::Vector3d(end_yaw[1], 0, 0), Eigen::Vector3d(end_yaw[2], 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // 生成 yaw / yawdot / yawdotdot 三条样条，供 traj_server / controller 后续采样。
  local_data_.yaw_traj_.setUniformBspline(yaw, pp_.bspline_degree_, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  // 这份 path_yaw_ 更多是调试/可视化数据，记录离散 look-forward yaw waypoints。
  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_ = path_yaw;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "yaw time: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw,
    bool lookfwd, const double& relax_time) {
  /*
    单机探索主线更常用的 yaw 规划接口。

    和 planYaw() 相比，这里多了两个探索特有约束：
    - 给定明确的最终目标 yaw（通常面向选中的 viewpoint）
    - 支持末段 relax_time，不强制整条轨迹都严格 look-forward

    可以把它理解成：
    “前半段尽量朝飞行方向看，末尾再平滑转到最终观测 yaw。”
  */
  const int seg_num = 12;
  // 这里直接把整条 yaw 轨迹固定分成 12 段，因此 dt_yaw 由当前位置轨迹总时长决定。
  double dt_yaw = local_data_.duration_ / seg_num;  // time of B-spline segment
  Eigen::Vector3d start_yaw3d = start_yaw;
  std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d.transpose()
            << ", end: " << end_yaw << std::endl;

  // 先把起始 yaw 归一化到 [-pi, pi]，但后面为了连续性又会用 calcNextYaw() 展开回连续角。
  while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];

  // yaw 控制点矩阵，维度是 (seg_num + degree) x 1。
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // 用起始 yaw / yawdot / yawddot 先固定前 3 个控制点。
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  /*
    若开启 lookfwd，就在前半段沿位置轨迹提取若干 yaw waypoints。

    `relax_num = relax_time / dt_yaw` 的作用很关键：
    最后这几段不再施加 look-forward waypoint，
    留给优化器更多自由度去平滑转向最终 end_yaw。
  */
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  if (lookfwd) {
    const double forward_t = 2.0;
    const int relax_num = relax_time / dt_yaw;
    for (int i = 1; i < seg_num - relax_num; ++i) {
      double tc = i * dt_yaw;
      Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
      double tf = min(local_data_.duration_, tc + forward_t);
      Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
      Eigen::Vector3d pd = pf - pc;
      // 当前位置到前视点的位移方向，就是当前这一段期望对准的 yaw。
      Eigen::Vector3d waypt;
      if (pd.norm() > 1e-6) {
        waypt(0) = atan2(pd(1), pd(0));
        waypt(1) = waypt(2) = 0.0;
        calcNextYaw(last_yaw, waypt(0));
      } else
        waypt = waypts.back();

      last_yaw = waypt(0);
      waypts.push_back(waypt);
      waypt_idx.push_back(i);
    }
  }
  // 再把最终目标 yaw 固定为尾部边界状态。
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // 防御性调试：如果起终 yaw 相差接近 pi，往往意味着角度展开出了问题或目标选择过于激烈。
  if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
    ROS_ERROR("Yaw change rapidly!");
    std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
  }

  // // Interpolate start and end value for smoothness
  // for (int i = 1; i < seg_num; ++i)
  // {
  //   double tc = i * dt_yaw;
  //   Eigen::Vector3d waypt = (1 - double(i) / seg_num) * start_yaw3d + double(i) / seg_num *
  //   end_yaw3d;
  //   std::cout << "i: " << i << ", wp: " << waypt[0] << ", ";
  //   calcNextYaw(last_yaw, waypt(0));
  // }
  // std::cout << "" << std::endl;

  auto t1 = ros::Time::now();

  // 调用 yaw 优化器：平滑 + 起点约束 + 终点约束 + 中间 look-forward waypoint 约束。
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::WAYPOINTS;
  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw3d[0], 0, 0),
    Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // std::cout << "2: " << (ros::Time::now() - t1).toSec() << std::endl;

  // 更新 yaw 样条及其导数。这里直接使用三阶 yaw B-spline。
  // 这里不调用 updateTrajInfo()，因为位置轨迹没变，只需要补齐 yaw 相关字段即可。
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;

  // plan_data_.path_yaw_ = path;
  // plan_data_.dt_yaw_path_ = dt_yaw * subsp;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  /*
    把新的目标 yaw 展开成“与 last_yaw 连续”的角度表示。

    这是 yaw 规划里非常关键的一个小工具。
    因为角度天然是模 2pi 的：
    - +179 deg 和 -179 deg 在几何上几乎同向
    - 但数值上直接相减会像是转了 358 deg

    这个函数做的就是选取“最短转角”的那一支，
    避免 yaw waypoint 在数值上发生跳变。
  */
  // 先把 last_yaw 取模到 [-pi, pi] 范围，作为比较基准。
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
