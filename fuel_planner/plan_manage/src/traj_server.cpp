#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}
using fast_planner::NonUniformBspline;
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;
using fast_planner::PerceptionUtils;

/*
  traj_server 在整套系统里的定位很关键：

  1. 上游 planner / FSM 并不直接给控制器发“当前时刻命令”，
     而是先发布一整条未来轨迹 `/planning/bspline`
  2. traj_server 收到后，把消息重新还原成内部 B-spline 对象
  3. 然后用高频定时器按“当前时间 - 轨迹起始时间”去采样
  4. 最终把某一时刻的期望状态发布为 `quadrotor_msgs::PositionCommand`
     给控制器或仿真器执行

  所以它本质上是：
  “整条轨迹消息” -> “当前时刻位置/速度/加速度/yaw 命令”
  的桥接节点。

  如果说 planner_manager 负责“算轨迹”，
  那 traj_server 负责“按时间把轨迹展开成连续控制命令”。
*/

// 对外发布器：
// - cmd_vis_pub : 画当前 FoV、箭头等调试可视化
// - pos_cmd_pub : 真正发给控制器的期望状态命令
// - traj_pub    : 画整条已执行/期望执行轨迹
ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;

// 最近一次真实里程计，仅用于记录实际飞行轨迹，不参与轨迹采样。
nav_msgs::Odometry odom;

// 复用的一帧 PositionCommand 消息。每次 cmdCallback 填完字段后直接发布。
quadrotor_msgs::PositionCommand cmd;

/*
  最近一次收到并正在执行的轨迹缓存。

  traj_ 里按固定槽位存多条样条，后面 cmdCallback 直接按下标取：
  - traj_[0] : 位置 p(t)
  - traj_[1] : 速度 v(t) = p'(t)
  - traj_[2] : 加速度 a(t) = p''(t)
  - traj_[3] : yaw(t)
  - traj_[4] : yawdot(t)
  - traj_[5] : jerk(t) = p'''(t)
*/
vector<NonUniformBspline> traj_;

// 当前轨迹的总时长。超出这个时长后，不再沿轨迹前进，而是保持末端状态。
double traj_duration_;

// 当前轨迹的生效时刻。cmdCallback 用 `now - start_time_` 得到轨迹时间参数 t_cur。
ros::Time start_time_;

// 当前正在执行的轨迹编号。用于拒绝乱序到达的旧轨迹。
int traj_id_;

// 可视化 marker 的 id，便于不同实验区分轨迹显示。
int pub_traj_id_;

// 感知工具类，主要用于根据当前位置和 yaw 画出当前相机 FoV。
shared_ptr<PerceptionUtils> percep_utils_;

// 是否已经收到至少一条轨迹。没轨迹前，cmdCallback 不发布采样命令。
bool receive_traj_ = false;

// 重规划切换保护时间。收到 replan 通知时，会把旧轨迹提前截断。
double replan_time_;

// 两类轨迹记录：
// - traj_cmd_  : 本节点实际发布过的“期望轨迹”
// - traj_real_ : 里程计返回的“真实飞行轨迹”
// 当前默认主要画 traj_cmd_。
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// 统计整段飞行表现时用到的时间戳与能量代理量。
// 这里 energy 不是严格物理能耗，而是用 jerk^2 累积出来的一个平滑代价近似。
ros::Time start_time, end_time, last_time;
double energy;

// 回环校正相关量。
// 某些实验里，规划轨迹在 world 系，而控制/里程计在校正后的 odom 系，
// 这里可用外部给出的 `pg_T_vio` 做坐标修正。
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

double calcPathLength(const vector<Eigen::Vector3d>& path) {
  // 简单累加折线长度，用于统计“这次飞了多长”。
  if (path.empty()) return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  /*
    用 SPHERE_LIST 把一条离散轨迹画出来。

    这里的 path 不是 B-spline 控制点，而是已经离散采样好的点列。
    典型输入是 traj_cmd_，也就是“traj_server 实际发出去的命令轨迹”。
  */
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  /*
    画当前视场角 FoV 的线框。

    PerceptionUtils::getFOV() 会给出成对的线段端点：
    - list1[i]
    - list2[i]
    每对点构成 LINE_LIST 的一条边。
  */
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0) return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  // 一个通用箭头绘制函数，常拿来画速度、加速度、yaw 方向等向量。
  // 当前主流程里默认没打开，但调试控制命令时很有用。
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void replanCallback(std_msgs::Empty msg) {
  /*
    收到“即将重规划”的通知后，不是立刻把旧轨迹硬切断，
    而是给旧轨迹留一个很短的缓冲时间，然后提前结束。

    这样做的目的是让轨迹切换更平滑：
    - planner/FSM 正在算新轨迹
    - 旧轨迹还能再飞很短一段
    - 新轨迹准备好后再接上

    `traj_duration_` 在这里被缩短，相当于告诉 cmdCallback：
    “旧轨迹只采样到 t_stop 为止，后面不要继续走了。”
  */
  const double time_out = 0.3;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  // 开始一轮新的实验/轨迹时，清空之前积累的期望轨迹和真实轨迹可视化缓存。
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  /*
    仅记录真实飞行轨迹，不参与 planner 轨迹求值。

    child_frame_id 为 "X"/"O" 的消息这里直接丢掉，
    一般是为了跳过某些无效或特殊来源的里程计帧。
  */
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
  odom = msg;
  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void pgTVioCallback(geometry_msgs::Pose msg) {
  /*
    接收外部提供的回环校正变换。

    可以把它理解成：
    当前规划坐标系 world 相对于控制/里程计坐标系的一个修正量。
    若 `isLoopCorrection=true`，后面发命令前会把位置/速度/加速度/yaw
    都转换一次。
  */
  Eigen::Quaterniond q =
      Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg.position.x, msg.position.y, msg.position.z;

  // cout << "R_loop: " << R_loop << endl;
  // cout << "T_loop: " << T_loop << endl;
}

void visCallback(const ros::TimerEvent& e) {
  // 低频可视化回调。当前默认画的是“期望执行轨迹” traj_cmd_。
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(1, 0, 0, 1), pub_traj_id_);
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), pub_traj_id_);
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 0, 1, 1), pub_traj_id_);

  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);
}

void bsplineCallback(const bspline::BsplineConstPtr& msg) {
  /*
    这是 traj_server 最重要的输入接口：
    上游 FSM / planner 发布 `/planning/bspline` 后，
    这里把消息反序列化回内部 B-spline 对象。

    注意这里还没有发布控制命令，只是“缓存整条轨迹”。
    真正采样并发 `/position_cmd` 是 cmdCallback 的事。
  */

  // 新轨迹必须严格比旧轨迹编号大，避免网络延迟导致旧消息覆盖新消息。
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("out of order bspline.");
    return;
  }

  // 先恢复位置 B-spline 的控制点和 knot 向量。
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // yaw 单独是一条 1 维 B-spline，knot span 由上游显式给出 yaw_dt。
  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);

  // start_time_ 决定“这条轨迹什么时候开始执行”。
  // 后面 cmdCallback 用 now - start_time_ 算当前应该采到轨迹的哪一段。
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  /*
    把后面高频采样会用到的导数一起预先算好并缓存。

    这样 cmdCallback 每次只需要 evaluateDeBoorT(t_cur)，
    不用再动态求导，逻辑更清楚，实时开销也更稳定。
  */
  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  // 只在第一次收到轨迹时记录“整次飞行开始时间”，便于做总时间统计。
  if (start_time.isZero()) {
    ROS_WARN("start flight");
    start_time = ros::Time::now();
  }
}

void cmdCallback(const ros::TimerEvent& e) {
  /*
    高频命令发布回调。

    它是整个 traj_server 的核心执行循环：
    - 根据当前时刻算出 t_cur
    - 在缓存好的 B-spline 上采样 pos/vel/acc/yaw/yawdot
    - 打包成 PositionCommand
    - 发给控制器
  */

  // 没收到轨迹前，什么都不发。
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();

  // t_cur 是轨迹内部时间参数，也就是“当前执行到了这条轨迹的第几秒”。
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // 正常执行区间：直接按当前轨迹时间采样。
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  } else if (t_cur >= traj_duration_) {
    /*
      轨迹已经走完：
      不再沿轨迹继续前进，而是保持末端位置和末端 yaw，
      同时把速度/加速度清零，相当于“悬停在终点”。
    */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;

    // 周期性打印整段飞行统计信息。
    double len = calcPathLength(traj_cmd_);
    double flight_t = (end_time - start_time).toSec();
    ROS_WARN_THROTTLE(2, "flight time: %lf, path length: %lf, mean vel: %lf, energy is: % lf ", flight_t,
                      len, len / flight_t, energy);
  } else {
    // t_cur < 0 表示轨迹生效时刻还没到。
    // 理论上上层 FSM 会尽量等到 start_time 再真正切换，这里仍保留防御性提示。
    cout << "[Traj server]: invalid time." << endl;
  }

  if (isLoopCorrection) {
    // 若启用回环校正，则把命令从规划坐标系转换到控制/里程计所用坐标系。
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
  }

  /*
    这里才是真正对控制器的输出接口。

    quadrotor_msgs::PositionCommand 里包含：
    - position
    - velocity
    - acceleration
    - yaw
    - yaw_dot

    也就是说，traj_server 输出的不是“整条轨迹”，
    而是“这一时刻控制器应该跟踪的期望状态”。
  */
  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub.publish(cmd);

  // 一些命令向量可视化接口，默认注释掉，需要时可打开看速度/加速度/yaw 方向。
  // Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  // drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  // 把当前命令位姿同步给感知工具类，用来绘制当前相机 FoV。
  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  /*
    记录本节点发出去的期望轨迹，用于：
    - 可视化
    - 统计路径长度
    - 估算 jerk 代价
  */
  if (traj_cmd_.size() == 0) {
    // 第一个采样点直接塞进去。
    traj_cmd_.push_back(pos);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // 与上一个点不同才记录，避免重复点把可视化和统计弄得太密。
    traj_cmd_.push_back(pos);
    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    end_time = ros::Time::now();
  }
  last_time = time_now;

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

void test() {
  /*
    这是一个离线测试函数，用来验证：
    多项式轨迹 -> B-spline 拟合 -> PositionCommand 发布
    这一整条链是否正常。

    主流程默认不会调用它。
  */
  // Test B-spline
  // Generate the first B-spline's control points from a sin curve
  vector<Eigen::Vector3d> samples;
  const double dt1 = M_PI / 6.0;
  for (double theta = 0; theta <= 2 * M_PI; theta += dt1) {
    Eigen::Vector3d sample(theta, sin(theta), 1);
    samples.push_back(sample);
  }
  Eigen::MatrixXd points(samples.size(), 3);
  for (int i = 0; i < samples.size(); ++i)
    points.row(i) = samples[i].transpose();

  Eigen::VectorXd times(samples.size() - 1);
  times.setConstant(dt1);
  times[0] += dt1;
  times[times.rows() - 1] += dt1;
  Eigen::Vector3d zero(0, 0, 0);

  PolynomialTraj poly;
  PolynomialTraj::waypointsTraj(points, zero, zero, zero, zero, times, poly);

  const int degree = 5;
  double duration = poly.getTotalTime();
  vector<Eigen::Vector3d> traj_pts;
  for (double ts = 0; ts <= duration; ts += 0.01)
    traj_pts.push_back(poly.evaluate(ts, 0));
  // displayTrajWithColor(traj_pts, 0.05, Eigen::Vector4d(1, 0, 0, 1), 99);

  // Fit the polynomialw with B-spline
  const int seg_num = 30;
  double dt = duration / seg_num;
  vector<Eigen::Vector3d> point_set, boundary_der;
  for (double ts = 0; ts <= 1e-3 + duration; ts += dt)
    point_set.push_back(poly.evaluate(ts, 0));

  boundary_der.push_back(poly.evaluate(0, 1));
  boundary_der.push_back(poly.evaluate(duration, 1));
  boundary_der.push_back(poly.evaluate(0, 2));
  boundary_der.push_back(poly.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, boundary_der, degree, ctrl_pts);
  NonUniformBspline fitted(ctrl_pts, degree, dt);

  traj_pts.clear();
  double duration2 = fitted.getTimeSum();
  for (double ts = 0; ts <= duration2; ts += 0.01)
    traj_pts.push_back(fitted.evaluateDeBoorT(ts));

  vector<Eigen::Vector3d> ctrl_pts_vec;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    Eigen::Vector3d pr = ctrl_pts.row(i).transpose();
    ctrl_pts_vec.push_back(pr);
  }
  displayTrajWithColor(ctrl_pts_vec, 0.1, Eigen::Vector4d(1, 1, 0, 1), 98);
  displayTrajWithColor(traj_pts, 0.05, Eigen::Vector4d(1, 0, 0, 1), 99);

  auto vel = fitted.getDerivative();
  auto acc = vel.getDerivative();

  ros::Duration(0.1).sleep();

  // Pub the traj
  auto t1 = ros::Time::now();
  double tn = (ros::Time::now() - t1).toSec();
  while (tn < duration && ros::ok()) {
    // Eigen::Vector3d p = bspline.evaluateDeBoorT(tn);
    // Eigen::Vector3d v = vel.evaluateDeBoorT(tn);
    // Eigen::Vector3d a = acc.evaluateDeBoorT(tn);
    Eigen::Vector3d p = fitted.evaluateDeBoorT(tn);
    Eigen::Vector3d v = vel.evaluateDeBoorT(tn);
    Eigen::Vector3d a = acc.evaluateDeBoorT(tn);

    cmd.header.stamp = ros::Time::now();
    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    pos_cmd_pub.publish(cmd);

    ros::Duration(0.02).sleep();
    tn = (ros::Time::now() - t1).toSec();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  /*
    输入接口：
    - planning/bspline : 上游 planner/FSM 发布的一整条 B-spline 轨迹
    - planning/replan  : 通知当前轨迹准备被新轨迹替换
    - planning/new     : 通知开始一轮新的记录/实验
    - /odom_world      : 真实里程计，用于记录实际轨迹
    - /loop_fusion/pg_T_vio : 回环校正变换
  */
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber pg_T_vio_sub = node.subscribe("/loop_fusion/pg_T_vio", 10, pgTVioCallback);

  /*
    输出接口：
    - planning/position_cmd_vis : 命令/FoV 可视化
    - /position_cmd             : 发给控制器或仿真器的当前位置命令
    - planning/travel_traj      : 已执行轨迹可视化
  */
  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  // cmd_timer 是主执行循环，100 Hz 按轨迹采样。
  // vis_timer 低频刷新可视化，避免 marker 发布过于频繁。
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("loop_correction/isLoopCorrection", isLoopCorrection, false);

  Eigen::Vector3d init_pos;
  nh.param("traj_server/init_x", init_pos[0], 0.0);
  nh.param("traj_server/init_y", init_pos[1], 0.0);
  nh.param("traj_server/init_z", init_pos[2], 0.0);

  ROS_WARN("[Traj server]: init...");
  ros::Duration(1.0).sleep();

  // 控制器增益直接写进 PositionCommand，供下游 so3_control 使用。
  cmd.kx = { 5.7, 5.7, 6.2 };
  cmd.kv = { 3.4, 3.4, 4.0 };

  std::cout << start_time.toSec() << std::endl;
  std::cout << end_time.toSec() << std::endl;

  // 初始化一帧“就绪但静止”的命令，作为系统启动时的默认控制输出。
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = init_pos[0];
  cmd.position.y = init_pos[1];
  cmd.position.z = init_pos[2];
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(nh));

  // test();
  /*
    启动时先做一个很短的上升-下降动作。

    这段不属于 FUEL 的探索算法本体，
    更像是仿真初始化/解锁动作，用来让四旋翼先进入稳定可控状态。
  */
  for (int i = 0; i < 100; ++i) {
    cmd.position.z += 0.01;
    pos_cmd_pub.publish(cmd);
    ros::Duration(0.01).sleep();
  }
  for (int i = 0; i < 100; ++i) {
    cmd.position.z -= 0.01;
    pos_cmd_pub.publish(cmd);
    ros::Duration(0.01).sleep();
  }
  // ros::Duration(1.0).sleep();
  // for (int i = 0; i < 100; ++i)
  // {
  //   cmd.position.x -= 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  // 到这里后，真正进入常驻运行：
  // 等待上游轨迹 -> 高频采样 -> 发布位置命令。
  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
