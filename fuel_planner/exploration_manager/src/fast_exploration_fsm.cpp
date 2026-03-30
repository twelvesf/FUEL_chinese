
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {

/*
  FastExplorationFSM 是单机探索 demo 的“总控层”。

  它自己不做 frontier 搜索、路径搜索或轨迹优化，而是负责：
  1. 等 odom 就绪
  2. 等外部触发开始探索
  3. 在合适的起始状态下调用 FastExplorationManager 规划
  4. 把规划结果封装成 /planning/bspline 发给 traj_server
  5. 监控执行过程，并在合适时机触发重规划

  对照 office demo 的主链路，这一层大致位于：
  /waypoint_generator/waypoints
    -> FastExplorationFSM
    -> FastExplorationManager
    -> /planning/bspline
    -> traj_server
    -> /planning/pos_cmd
    -> 控制与仿真

  所以读这个文件时，可以把它理解成：
  “什么时候开始规划、从哪里开始规划、什么时候要重规划”。
*/

void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /* FSM 参数：三类重规划阈值 + 动态重规划时向前取样的时间偏移 */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* 初始化下游主模块：探索管理器与可视化器 */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  // 规划器管理器实际由 exploration manager 持有，这里只是拿指针做执行监控和碰撞检查。
  planner_manager_ = expl_manager_->planner_manager_;

  // 初始状态：未拿到 odom，不能开始探索。
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };

  // static_state_ = true 表示当前起飞器处于静止 / 悬停状态，可以直接用 odom 作为规划起点。
  // 如果是 false，则表示正在执行旧轨迹，重规划要从旧轨迹未来一点的位置继续接。
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  /*
    定时器与 ROS 接口：
    - exec_timer_     : FSM 主循环，100 Hz
    - safety_timer_   : 执行期碰撞检查，20 Hz
    - frontier_timer_ : 等待触发 / 已完成时也持续刷新 frontier，可用于可视化观察地图变化
  */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  // 这里的触发不是直接订阅 RViz 的 /move_base_simple/goal，
  // 而是订阅 waypoint_generator 处理后的 /waypoint_generator/waypoints。
  // 在 office demo 中，这条 Path 主要被当作“开始探索”的信号，而不是最终飞行目标。
  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);

  // FSM 内部统一使用 /odom_world 这个名字，上游在 launch 中 remap 到真实 odom 来源。
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  // 给 traj_server 的两个控制信号：
  // - /planning/replan：当前执行轨迹要被新规划截断
  // - /planning/bspline：新的位置 / yaw B-spline
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // 只有 odom 准备好之后，才能知道起点位置、速度和朝向。
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // 一旦 odom 就绪，就进入“等待触发”状态。
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // 在这个状态下 FSM 不主动规划，只等 triggerCallback 把它推入 PLAN_TRAJ。
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      // 当前没有新的 frontier 需要访问，探索结束。
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      /*
        规划前先确定“这一次规划从哪里起步”。

        两种情况：
        1. static_state_ = true
           当前处于悬停 / 静止，直接用实时 odom 作为起点状态。
        2. static_state_ = false
           当前还在执行上一条轨迹，重规划不能从“当前采样时刻”硬切，
           而是从旧轨迹未来 replan_time 秒的位置接上，保证轨迹连续性。
      */
      if (fd_->static_state_) {
        // 静止起步：当前位置、当前速度、当前 yaw 直接来自 odom。
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // 动态重规划：从旧轨迹未来一个小时间偏移处继续规划，避免轨迹断裂。
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // 提前通知 traj_server：旧轨迹可能要被截断，准备切换到新轨迹。
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        // 规划成功后不立即执行，先进入 PUB_TRAJ，等待新轨迹到达生效时刻再发布。
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        // Frontier 已经都覆盖完了，不再继续规划。
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // 本次规划失败，但探索并未结束；保持在 PLAN_TRAJ，下一次定时器继续尝试。
        ROS_WARN("plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      /*
        新轨迹不是收到就立刻发，而是等到它的 start_time 已经到达。
        这是为了让轨迹消息时间戳和实际执行时刻对齐。
      */
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);

        // 一旦开始执行轨迹，就不再是静止状态；后续若重规划，要从旧轨迹未来状态接续。
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        // 可视化放到独立线程里做，避免阻塞 FSM 主循环。
        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      // 执行阶段主要做“是否需要重新规划”的判定。
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // 条件 1：轨迹快执行完了，需要提前接下一段探索轨迹。
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }

      // 条件 2：当前要去的 frontier 在飞行过程中已经被看完了，继续沿旧轨迹飞没有意义。
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }

      // 条件 3：周期性重规划。
      // FUEL 的默认主线会定期重规划，以便及时吸收最新 frontier / 地图变化。
      // classic_ 为 true 时对应老基线方法，不走这条周期重规划逻辑。
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  // 规划出的新轨迹会尽量安排在“当前时刻 + replan_time”附近开始生效。
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  // 真正的 frontier 搜索、tour 规划、局部 refinement、轨迹优化都在 manager 里完成。
  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;

    // 如果 time_r 已经过了，就立刻开始；否则按 time_r 这个未来时刻启动。
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    /*
      这里做的不是“再规划一次轨迹”，而是把 planner_manager_ 内部已经准备好的轨迹
      封装成 ROS 消息 bspline::Bspline，交给 traj_server 去按时间采样执行。

      这一步是 FSM 和 traj_server 之间的核心接口转换。
    */
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    // 位置 B-spline 控制点
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    // yaw B-spline 控制点。yaw_dt 是 yaw 样条的 knot span。
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  // 这里只负责把 exploration manager / planner manager 内部缓存的数据画出来，
  // 不参与任何规划逻辑。
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // 画当前 frontier cluster。每个 cluster 一种颜色。
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // 红线是当前局部执行轨迹的 B-spline，可和 traj_server 的执行轨迹可视化对照着看。
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  // 降频执行：frontier 搜索相对更重，这里每 5 次定时器才真正刷新一次。
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    /*
      在“等待启动”和“探索结束”这两个状态下，系统仍然持续根据最新地图刷新 frontier。
      这样做有两个目的：
      1. RViz 中可以实时看到 frontier 的形成和消失
      2. 一旦触发探索，frontier 结构已经是最新的，不用再从零开始准备
    */
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    // 这三步对应 frontier 增量维护的主流程：
    // 搜索 / 筛选待访问 frontier / 更新代价矩阵。
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    // 从 frontier_finder 拷贝一份结果到 ExplorationData，供可视化使用。
    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // 画 frontier 和边界盒。当前默认只画体素块，边界盒代码保留但注释掉了。
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {
  /*
    这个回调最容易读错。

    它订阅的是 /waypoint_generator/waypoints，而不是 planner 的最终目标点。
    在当前 office demo 中，FSM 实际并不会读取整条 Path 去“逐点飞过去”。
    它只把这条消息当作“开始探索”的触发信号。

    也就是说：
    RViz 2D Nav Goal -> waypoint_generator -> Path
    对 FastExplorationFSM 来说，最重要的意义只是：有人点了一下，现在可以开始探索了。
  */

  // z < -0.1 在 waypoint_generator 的语义里通常表示无效 / 结束类信号，这里直接忽略。
  if (msg->poses[0].pose.position.z < -0.1) return;

  // 只有在 WAIT_TRIGGER 状态下才接受启动；其余状态下再次点击不改变当前流程。
  if (state_ != WAIT_TRIGGER) return;

  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // 执行期间持续做轨迹碰撞前瞻检查，一旦发现未来轨迹不安全就立即重规划。
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  // 缓存最新 odom，供 INIT 判定、静止起步规划和 yaw 提取使用。
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  // 用机体系 x 轴在 world 中的投影方向来恢复 yaw。
  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  // 统一的状态切换打印，方便在终端中对照运行时序。
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace fast_planner
