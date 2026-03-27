# FUEL 单机探索代码精读笔记

这份笔记按 `office.pcd` 默认仿真演示来读单机 FUEL，不展开 RACER 多机代码，也不优先读 `topo_replan`、`kino_replan` 这些旧 demo。

阅读目标不是把所有类都过一遍，而是先讲清楚这一条闭环：

`RViz 2D Nav Goal -> waypoint_generator -> FastExplorationFSM -> FastExplorationManager -> FrontierFinder / FastPlannerManager -> traj_server -> so3_control / simulator -> local_sensing -> map_ros / SDFMap -> frontier 更新 -> 重新规划`

## 1. 入口锚点

建议先盯这 3 个文件：

- `fuel_planner/exploration_manager/launch/exploration.launch`
- `fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp`
- `fuel_planner/exploration_manager/src/fast_exploration_manager.cpp`

如果只想先建立“大脑里的总图”，顺序建议是：

1. `exploration.launch`
2. `algorithm.xml`
3. `simulator.xml`
4. `fast_exploration_fsm.cpp`
5. `fast_exploration_manager.cpp`
6. `active_perception/frontier_finder.cpp`
7. `plan_manage/planner_manager.cpp`
8. `plan_env/sdf_map.cpp` 和 `plan_env/map_ros.cpp`
9. `plan_manage/traj_server.cpp`

## 2. 启动链路与节点拓扑

### 2.1 `rviz.launch` 做了什么

`fuel_planner/exploration_manager/launch/rviz.launch` 很简单：

- 启动 `rviz`
- 加载 `plan_manage/config/traj.rviz`
- 发布静态 TF：`world -> navigation`

它只负责可视化，不参与规划逻辑。

### 2.2 `exploration.launch` 拉起了什么

`fuel_planner/exploration_manager/launch/exploration.launch` 做了 4 件事：

1. 通过 `algorithm.xml` 启动主算法节点 `exploration_node`
2. 启动 `traj_server`
3. 启动 `waypoint_generator`
4. 通过 `simulator.xml` 启动地图、传感器、控制器和动力学仿真

默认 office demo 的核心节点可以按职责分成下面几组。

算法本体：

- `exploration_node`
- `traj_server`

触发与人机交互：

- `waypoint_generator`
- `rviz`

仿真执行链：

- `quadrotor_simulator_so3`
- `so3_control`
- `so3_disturbance_generator`
- `odom_visualization`

环境与感知：

- `map_pub`
- `pcl_render_node`

### 2.3 关键话题是谁订阅的

先盯 6 个核心接口：

- `/move_base_simple/goal`
- `/waypoint_generator/waypoints`
- `/odom_world`
- `/map_ros/depth` 或 `/map_ros/cloud`
- `/planning/bspline`
- `/planning/pos_cmd`

它们在默认 demo 里的去向如下。

`/move_base_simple/goal`

- 由 RViz 的 `2D Nav Goal` 工具发布
- 在 `exploration.launch` 中被 remap 到 `waypoint_generator` 的私有话题 `~goal`
- `waypoint_generator` 的 `goal_callback()` 接收它

`/waypoint_generator/waypoints`

- 由 `waypoint_generator` 发布
- `FastExplorationFSM::triggerCallback()` 订阅它
- 在 FUEL 探索 demo 中，这个话题本质上是“启动探索”的触发器

`/odom_world`

- `exploration.launch` 里 remap 到 `/state_ukf/odom`
- `FastExplorationFSM` 用它拿当前位姿速度
- `traj_server` 也订阅它，用于记录执行轨迹和碰撞检查的时序对齐

`/map_ros/depth` 和 `/map_ros/cloud`

- `algorithm.xml` 把它们 remap 到 `/pcl_render_node/depth` 和 `/pcl_render_node/cloud`
- `MapROS` 同时准备了两套同步订阅
- 默认 `office` 演示主要走深度图分支，因为 `pcl_render_node` 默认发布的是 `/pcl_render_node/depth` 和 `/pcl_render_node/sensor_pose`

`/planning/bspline`

- `FastExplorationFSM` 在规划成功后发布
- `traj_server` 订阅它并解析为位置 B-spline 和 yaw B-spline

`/planning/pos_cmd`

- `traj_server` 输出位置控制命令到这个话题
- `simulator.xml` 中 `so3_control` 把它 remap 为自己的 `~position_cmd`
- 再由 `so3_control` 转成 `so3_cmd` 给动力学仿真器

### 2.4 参数大类怎么分

`algorithm.xml` 的参数其实已经把整套系统切成了几个模块：

- `sdf_map/*`：占据栅格、更新框、膨胀、概率融合
- `map_ros/*`：深度图/点云融合与地图可视化
- `fsm/*`：重规划阈值和时间
- `exploration/*`：局部 refinement、TSP、yaw/time surrogate cost
- `frontier/*`：frontier 聚类、候选 viewpoint、可见性阈值
- `perception_utils/*`：视场角和感知距离
- `heading_planner/*`：偏航规划
- `manager/*`：规划总开关和动力学限制
- `search/*`：kinodynamic A*
- `astar/*`：几何 A*
- `optimization/*` 和 `bspline/*`：B-spline 优化

论文里的 Fig.2 是系统视图；`algorithm.xml` 则是系统视图在代码里的参数映射。

## 3. 触发机制与探索总控 FSM

### 3.1 主入口只做一件事

`exploration_node.cpp` 很薄：

- 初始化 ROS
- 构造 `FastExplorationFSM`
- `init(nh)`
- `ros::spin()`

真正的逻辑都在 `FastExplorationFSM`。

### 3.2 状态机结构

`FastExplorationFSM` 的状态是：

- `INIT`
- `WAIT_TRIGGER`
- `PLAN_TRAJ`
- `PUB_TRAJ`
- `EXEC_TRAJ`
- `FINISH`

它内部主要维护 3 个对象：

- `planner_manager_`
- `expl_manager_`
- `visualization_`

也就是：

- 运动规划与轨迹优化
- 探索决策
- 可视化

### 3.3 状态切换逻辑

`INIT`

- 等待 `/odom_world`
- 没有 odom 就一直停在这里

`WAIT_TRIGGER`

- 已经有 odom，但还没开始探索
- 此时 `frontierCallback()` 会周期性搜 frontier 并画出来

`PLAN_TRAJ`

- 准备起始状态
- 如果当前静止，就直接从 odom 状态规划
- 如果正在执行旧轨迹，就从“当前时间往后 `replan_time_` 秒”的轨迹状态重规划

`PUB_TRAJ`

- 规划成功后，组装 `bspline::Bspline` 消息并等待生效时刻

`EXEC_TRAJ`

- 轨迹执行中
- 同时盯着 3 类重规划触发条件

`FINISH`

- 没有可覆盖 frontier 了

### 3.4 三类重规划触发条件

`EXEC_TRAJ` 中有 3 种典型重规划：

1. 当前轨迹快执行完了
- `time_to_end < replan_thresh1_`

2. 正在去的那个 frontier 已被覆盖
- `t_cur > replan_thresh2_`
- `frontier_finder_->isFrontierCovered() == true`

3. 周期性重规划
- `t_cur > replan_thresh3_`
- 只对非 classic 模式生效

此外还有一条安全链：

- `safetyCallback()` 调 `planner_manager_->checkTrajCollision()`
- 一旦未来轨迹段碰撞，直接切回 `PLAN_TRAJ`

### 3.5 为什么 2D Nav Goal 只是“启动按钮”

这是默认 demo 里最容易误解的地方。

`exploration.launch` 把 `waypoint_generator` 设成：

- `waypoint_type = point`

而 `waypoint_generator` 的 `goal_callback()` 在 `point` 模式下并不会使用你在 RViz 点出来的坐标，而是直接调用 `sample_waypoints.h` 里的 `point()` 返回一串固定 path。

更关键的是：

- `FastExplorationFSM::triggerCallback()` 根本不读取 path 里的目标点
- 它只检查 `msg->poses[0].pose.position.z` 是否有效
- 然后把状态从 `WAIT_TRIGGER` 切到 `PLAN_TRAJ`

所以在单机探索 demo 里：

- 点击 `2D Nav Goal` 的意义是“发出开始探索的信号”
- 不是“让无人机飞到我点击的位置”

这也解释了为什么 README 里说“用 2D Nav Goal 触发探索”，而不是“指定探索目标”。

## 4. 仿真输入栈与执行输出栈

### 4.1 从点击到触发

链路是：

- RViz 发布 `/move_base_simple/goal`
- `waypoint_generator` 订阅 `~goal`
- `waypoint_generator` 发布 `~waypoints`
- remap 后变成 `/waypoint_generator/waypoints`
- `FastExplorationFSM::triggerCallback()` 收到后进入 `PLAN_TRAJ`

### 4.2 从 B-spline 到控制命令

规划成功后的链路是：

- `FastExplorationFSM` 发布 `/planning/bspline`
- `traj_server` 解析 B-spline
- `traj_server` 定时发布 `/planning/pos_cmd`
- `so3_control` 订阅 `position_cmd`
- `so3_control` 输出 `so3_cmd`
- `quadrotor_simulator_so3` 订阅 `cmd`

所以：

- `FastExplorationFSM` 负责“把规划结果发出去”
- `traj_server` 负责“按时间采样轨迹”
- `so3_control` 负责“把位置命令变成姿态/力矩命令”
- `quadrotor_simulator_so3` 负责“积分动力学”

### 4.3 里程计是怎么来的

默认仿真中：

- `quadrotor_simulator_so3` 发布 `~odom`
- remap 到 `/visual_slam/odom`
- `so3_disturbance_generator` 基于它生成带误差的 `/state_ukf/odom`
- `exploration.launch` 再把 `/state_ukf/odom` 作为 `odom_topic`

所以算法看到的 odom 不是仿真真值，而是经过一层扰动/滤波后的状态估计。

### 4.4 传感器链路怎么来的

环境输入链路是：

- `map_pub` 读取 `office.pcd`
- 连续发布 `/map_generator/global_cloud`
- `pcl_render_node` 订阅这个全局点云和 odometry
- 按当前相机位姿渲染深度图
- 发布 `/pcl_render_node/depth`
- 同时发布 `/pcl_render_node/sensor_pose`

然后算法侧：

- `MapROS` 同步接收 depth 和 sensor pose
- 做投影、raycasting、占据更新、膨胀和 ESDF 更新

默认 office demo 主要走的是：

- `global cloud -> synthetic depth -> map_ros -> occupancy / ESDF`

而不是“把全局点云直接喂给探索算法”。

### 4.5 哪些属于算法本体，哪些只是仿真支撑

算法本体：

- `FastExplorationFSM`
- `FastExplorationManager`
- `FrontierFinder`
- `FastPlannerManager`
- `SDFMap / MapROS / EDTEnvironment`

仿真支撑：

- `map_pub`
- `pcl_render_node`
- `so3_control`
- `quadrotor_simulator_so3`
- `so3_disturbance_generator`
- `odom_visualization`
- `waypoint_generator`

## 5. 地图与 ESDF

### 5.1 `SDFMap` 是地图核心

`SDFMap` 管三套关键数据：

- `occupancy_buffer_`：log-odds 占据
- `occupancy_buffer_inflate_`：膨胀后的占据
- `distance_buffer_`：ESDF 距离

它既是 frontier 检测的基础，也是轨迹优化的距离场基础。

### 5.2 `MapROS` 是 ROS 封装层

`MapROS` 做 3 件事：

1. 同步接收深度或点云加相机位姿
2. 调 `SDFMap::inputPointCloud()` 融合测量
3. 定时更新 ESDF 与发布可视化点云

默认用到的话题是：

- `/map_ros/depth`
- `/map_ros/pose`

点云分支也准备好了，但 default demo 基本不走。

### 5.3 深度图如何进占据地图

`MapROS::depthPoseCallback()` 的流程是：

1. 更新 `camera_pos_` 和 `camera_q_`
2. 把 depth image 转成点云
3. 调 `map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_)`
4. 如果有局部更新，就：
- `clearAndInflateLocalMap()`
- `esdf_need_update_ = true`

### 5.4 `updated box` 为什么重要

`SDFMap::inputPointCloud()` 每次融合时都会维护：

- `local_bound_min_ / local_bound_max_`
- `update_min_ / update_max_`

其中：

- `local_bound_*` 用来限制这次局部重膨胀和 ESDF 更新的体素范围
- `update_*` 用来告诉 frontier 模块“地图哪里刚变过”

这就是 frontier 能增量更新的根基。

如果没有这个 update box，frontier 模块就只能每次全图重扫。

### 5.5 ESDF 什么时候更新

ESDF 不是每收到一帧深度就立刻同步全图刷新，而是：

- 地图融合后只把 `esdf_need_update_` 置真
- `MapROS::updateESDFCallback()` 定时器检查它
- 只在局部更新框内做 3D EDT

这也是论文里“高频 planning”的工程基础之一。

### 5.6 后续模块为什么反复用这几个接口

后面所有模块都依赖这些基础查询：

- `isInBox()`
- `getOccupancy()`
- `getInflateOccupancy()`
- `getDistance()`
- `getUpdatedBox()`

它们分别承担：

- 探索空间边界限制
- 判断 frontier / unknown
- 判断安全碰撞与可见性遮挡
- 提供 ESDF 距离
- 只在变化区域增量更新 frontier

## 6. Frontier Information Structure 与增量 frontier 更新

### 6.1 论文里的 FIS 在代码里落哪了

论文 Table I 里的 FIS 并没有作为一个单独 `FIS` 类存在，而是分散落在 `Frontier` 结构和 `FrontierFinder` 的几个链表里。

`Frontier` 结构大致对应：

- `cells_`：cluster 内完整 frontier cells
- `filtered_cells_`：下采样后用于 viewpoint 评估
- `average_`：cluster 平均位置
- `box_min_ / box_max_`：AABB
- `viewpoints_`：覆盖该 cluster 的候选视点
- `paths_ / costs_`：与其他 frontier cluster 的连接路径和代价

也就是说：

- 论文里的 `Ci` 对应 `cells_`
- `pavg,i` 对应 `average_`
- `Bi` 对应 `box_min_ / box_max_`
- `VPi` 对应 `viewpoints_`
- `Lcost,i` 对应 `costs_` 加 `paths_`

### 6.2 增量更新的总流程

`searchFrontiers()` 的逻辑和论文 Sec.IV 几乎一一对应：

1. 取 `SDFMap` 的 `updated box`
2. 遍历旧 active frontier
3. 如果 frontier 的 AABB 和更新框重叠，并且 cluster 结构已变化，就删掉
4. 同样检查 `dormant_frontiers_`
5. 只在“更新框稍微膨胀后的区域”里重新扫描 frontier seed
6. 通过 region growing 扩展为 cluster
7. 对过大的 cluster 做 PCA split

这就不是“全图重新找 frontier”，而是“旧 frontier 局部失效 + 新 frontier 局部补上”。

### 6.3 `frontiers_ / dormant_frontiers_ / removed_ids_ / first_new_ftr_` 各干什么

`frontiers_`

- 当前可访问的 active frontier clusters
- 这些 cluster 至少有一个可用 viewpoint

`dormant_frontiers_`

- frontier 还在，但当前找不到合法 viewpoint
- 例如太挤、太靠 unknown、不可安全观察

`removed_ids_`

- 记录这轮从 active frontier 中删掉了哪些 cluster index
- 后续 `updateFrontierCostMatrix()` 用它来删除旧 cost list 里的对应项

`first_new_ftr_`

- 指向本轮新增 active frontier 的起点
- 用来把“旧 cluster 与新 cluster 的连边”和“新 cluster 之间的连边”分开做
- 这是增量 cost matrix 更新的关键游标

### 6.4 frontier 检测条件是什么

代码里的 frontier seed 条件和经典定义一致：

- 当前 voxel 是 `knownfree`
- 它至少邻接一个 `unknown`

具体判断由这些函数配合完成：

- `knownfree()`
- `isNeighborUnknown()`
- `expandFrontier()`

`expandFrontier()` 用 queue 做区域生长，最后把太小的 cluster 过滤掉。

### 6.5 为什么还要 PCA split

大 cluster 会把多个未知区域揉在一起，不利于后续全局 tour 和 viewpoint 选择。

所以 `splitHorizontally()` 会：

1. 对 `filtered_cells_` 求 2D 协方差
2. 取第一主轴
3. 沿主轴把 cluster 一分为二
4. 递归直到满足 `cluster_size_xy_`

这正对应论文里“把大 frontier 拆成更均匀的小 cluster”的操作。

### 6.6 候选 viewpoint 是怎么生成的

`sampleViewpoints()` 不是飞去 cluster 中心，而是：

1. 以 frontier 平均位置为中心
2. 在多个半径和多个角度上采样环状位置
3. 过滤掉：
- 不在 box 内
- 落在膨胀障碍中
- 离 unknown 太近
4. 对每个候选位置估一个平均 yaw
5. 用 `countVisibleCells()` 统计可见 frontier cells 数
6. 超过阈值才保留为 `Viewpoint`

`countVisibleCells()` 又会检查：

- 是否在 FOV 内
- 从 viewpoint 到 frontier cell 的射线是否被障碍或 unknown 挡住

### 6.7 什么时候认为 frontier 已经“被覆盖”

`isFrontierCovered()` 只检查和 `updated box` 有重叠的 frontier。

如果某个 cluster 中有足够比例的 cell 不再满足 frontier 条件，超过：

- `min_view_finish_fraction_ * cluster_size`

就返回 true。

FSM 正是靠这个信号在执行中触发 replan。

## 7. 全局 tour 与局部 viewpoint refinement

### 7.1 `planExploreMotion()` 是探索规划总入口

`FastExplorationManager::planExploreMotion()` 大致分 5 步：

1. `searchFrontiers()`
2. `computeFrontiersToVisit()`
3. 取 active frontiers 的 top viewpoints
4. 先做 global tour，再做 local refinement
5. 选出下一个 viewpoint 后调用轨迹规划

这正对应论文的层级结构。

### 7.2 Global tour 怎么做

先由 `FrontierFinder` 给出每个 frontier 的 top viewpoint。

然后 `findGlobalTour()`：

1. `updateFrontierCostMatrix()`
2. `getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat)`
3. 把代价矩阵写成 `single.tsp`
4. 调 `solveTSPLKH()`
5. 读 `single.txt` 的 tour 结果

所以论文里的 ATSP 在代码里不是抽象求解，而是：

- 显式写 TSPLIB 文件
- 调 LKH
- 再解析 solver 输出

这里调用 LKH 的原因很直接：

- frontier 数量变大后，遍历排列组合不现实
- LKH 对 TSP/ATSP 是成熟的启发式求解器
- 论文和代码都把它作为 coarse global ordering 的求解后端

### 7.3 局部 refinement 为什么只看前几个 frontier

`refine_local_` 打开后，代码不会对整条 global tour 的所有 viewpoint 联合优化，而只取：

- global tour 前缀中的若干 frontier
- 数量上限是 `refined_num_`
- 同时受 `refined_radius_` 限制

原因和论文一致：

- 远处 frontier 的视点细修意义不大
- 真正即将访问的一小段更值得花算力
- 这样局部图搜索规模可控，能保持高频 replanning

### 7.4 局部 refinement 是怎么建图的

`refineLocalTour()` 做的是一个分组 DAG 搜索：

1. 当前状态作为第一组
2. 每个 frontier 的多个候选 viewpoint 作为一组
3. 只连相邻组之间的边
4. `GraphSearch<ViewNode>::DijkstraSearch()` 求最优序列

这里 `ViewNode` 的边代价来自 `ViewNode::computeCost()`。

### 7.5 `ViewNode::computeCost()` 把哪些东西揉在一起

它的 cost 由两部分取最大值：

1. 平移代价
- 先 `searchPath(p1, p2, path)`
- 路径长度除以 `vm_`
- 如果当前已有速度，还叠加速度方向不一致惩罚 `w_dir_ * diff`

2. 偏航代价
- `abs(y2 - y1) / yd_`
- 自动 wrap 到最小角差

最后：

- `return max(pos_cost, yaw_cost);`

这就是论文里 time lower bound 加 motion consistency 的代码版落地。

## 8. 轨迹生成、yaw 规划与最短时间优化

### 8.1 什么时候走几何路径，什么时候走 kinodynamic A*

选出 `next_pos / next_yaw` 后，`planExploreMotion()` 先用普通 A* 找一条几何路径，再根据路径长度分 3 类：

很近：

- 路径长度 `< 1.5 m`
- 不走 kinodynamic A*
- 直接 `planExploreTraj()` 做基于 waypoint 的轨迹优化

很远：

- 路径长度 `> 5.0 m`
- 从几何路径上截一个中间目标
- 先飞向中间目标
- 仍走 `planExploreTraj()`

中距离：

- 直接 `kinodynamicReplan()`
- 精确飞向选中的 viewpoint

所以默认单机 FUEL 并不是“永远 kinodynamic A*”：

- 近和远都走更便宜的 geometric path + B-spline
- 中距离才用 kino search 精细打通

### 8.2 `planExploreTraj()` 做了什么

它的流程是：

1. 把离散路径点变成一条初始多项式轨迹
2. 对多项式轨迹离散采样
3. 参数化成 uniform B-spline
4. 调 `BsplineOptimizer::optimize()`
5. 生成 `local_data_.position_traj_`
6. `updateTrajInfo()`

### 8.3 `kinodynamicReplan()` 做了什么

它的流程是：

1. `KinodynamicAstar::search()`
2. 从 kino path 取 sample 和边界导数
3. 参数化成 B-spline
4. 用 `BsplineOptimizer` 做二次优化
5. `updateTrajInfo()`

也就是说：

- kinodynamic A* 先给可行初始化
- B-spline optimizer 再把它变成更平滑、更可执行的局部轨迹

### 8.4 “minimum-time trajectory” 在代码里到底落在哪里

论文里的 minimum-time 不是单个函数独立完成，而是几部分共同完成：

- `FastExplorationManager`
  - 给出目标 viewpoint
  - 计算 yaw 时间下界 `time_lb`

- `FastPlannerManager`
  - `planExploreTraj()` 或 `kinodynamicReplan()`
  - 把位置轨迹表示成 B-spline

- `BsplineOptimizer`
  - 在 cost function 中考虑 `MINTIME`
  - 真正做时间与轨迹形状优化

- `planYawExplore()`
  - 生成和位置轨迹对齐的 yaw B-spline

所以论文里的“minimum-time trajectory generation”在代码里是一个组合模块，不是单个 API。

### 8.5 `planYawExplore()` 如何做 look-forward

`planYawExplore(start_yaw, end_yaw, true, relax_time)` 的关键是：

1. 先按位置轨迹总时长把 yaw 分成若干段
2. 对每一段，取当前位置 `pc`
3. 再看未来 `forward_t` 秒后的位置 `pf`
4. 用 `pf - pc` 的方向作为 yaw waypoint
5. 末尾留一段 `relax_time` 不强行 look-forward
6. 最后再把终点 yaw 作为边界约束

这样 yaw 就不是死盯最终目标，而是尽量沿着未来飞行方向看过去。

### 8.6 `updateTrajInfo()` 之后谁来消费数据

`updateTrajInfo()` 只做 3 件事：

- 生成速度轨迹
- 生成加速度轨迹
- 更新 `duration_` 和 `traj_id_`

之后消费这些数据的有两路：

一路是 FSM：

- `callExplorationPlanner()` 从 `local_data_` 里取控制点和 knots
- 组装成 `/planning/bspline`

一路是内部重规划：

- `PLAN_TRAJ` 状态下如果从动态状态重规划
- 会直接在旧 `local_data_` 上按当前时刻往后采样新的起点状态

## 9. 可视化与 demo 现象映射

### 9.1 frontier 是谁画的

frontier cubes 主要由：

- `FastExplorationFSM::frontierCallback()`
- `FastExplorationFSM::visualize()`

调用：

- `PlanningVisualization::drawCubes(..., "frontier", ..., pub_id=4)`

对应话题：

- `/planning_vis/frontier`

### 9.2 规划轨迹是谁画的

规划得到的 B-spline 由：

- `FastExplorationFSM::visualize()`
- `PlanningVisualization::drawBspline()`

发布到：

- `/planning_vis/trajectory`

### 9.3 执行轨迹是谁画的

`traj_server` 会累计命令轨迹 `traj_cmd_`，并在 `visCallback()` 里发布到：

- `planning/travel_traj`

这个更像“执行参考轨迹”。

### 9.4 当前 FoV 是谁画的

当前 FoV 不是 `FastExplorationFSM` 画的，而是 `traj_server` 在 `cmdCallback()` 中：

1. 用当前命令位置和 yaw 设置 `PerceptionUtils`
2. `getFOV(l1, l2)`
3. `drawFOV(l1, l2)`

发布到：

- `planning/position_cmd_vis`

### 9.5 已建图体素是谁画的

`MapROS::publishMapLocal()` 和 `publishMapAll()` 发布：

- `/sdf_map/occupancy_local`
- `/sdf_map/occupancy_all`

这些是“已经被感知并融合进地图的占据体素”。

### 9.6 demo 里常见现象与发布者对照

彩色 frontier cubes：

- `/planning_vis/frontier`
- 发布者是 `PlanningVisualization`

规划 B-spline：

- `/planning_vis/trajectory`
- 发布者是 `PlanningVisualization`

执行参考轨迹：

- `planning/travel_traj`
- 发布者是 `traj_server`

当前 FoV：

- `planning/position_cmd_vis`
- 发布者是 `traj_server`

已建图占据体素：

- `/sdf_map/occupancy_local`
- `/sdf_map/occupancy_all`
- 发布者是 `MapROS`

输入深度图：

- `/pcl_render_node/depth`
- 发布者是 `pcl_render_node`

## 10. 按 demo 反查代码的验证点

这 4 个检查点可以作为你以后读代码时的固定 sanity check。

### 10.1 点击 `2D Nav Goal` 后发生了什么

应该确认：

- RViz 只是在 `/move_base_simple/goal` 上发了一条消息
- `waypoint_generator` 把它转成 `/waypoint_generator/waypoints`
- `FastExplorationFSM` 只把它当作触发信号
- 真正的探索目标不是点击坐标，而是 frontier planner 自己选出来的

### 10.2 地图更新后 frontier 为什么不是只追加

应该确认：

- `SDFMap` 会记录 `updated box`
- `FrontierFinder` 先删旧 frontier，再搜新 frontier
- `removed_ids_` 和 `first_new_ftr_` 支撑 cost matrix 的增量维护

### 10.3 规划成功后怎么走到控制命令

应该确认：

- 先有 `/planning/bspline`
- 再由 `traj_server` 采样成 `/planning/pos_cmd`
- 再由 `so3_control` 变成 `so3_cmd`
- 再送到 `quadrotor_simulator_so3`

### 10.4 执行中为什么会自动重规划

应该确认 3 条线都存在：

- 轨迹快结束
- 目标 frontier 被覆盖
- 安全检查发现未来轨迹碰撞

## 11. 论文到代码的最短映射

论文里的三层规划在代码里最直接的落点是：

全局 frontier 结构维护：

- `FrontierFinder::searchFrontiers()`
- `FrontierFinder::computeFrontiersToVisit()`
- `FrontierFinder::updateFrontierCostMatrix()`

全局 tour 与局部 viewpoint refinement：

- `FastExplorationManager::findGlobalTour()`
- `FastExplorationManager::refineLocalTour()`

最短时间局部轨迹生成：

- `FastPlannerManager::planExploreTraj()`
- `FastPlannerManager::kinodynamicReplan()`
- `FastPlannerManager::planYawExplore()`

如果只想抓论文主线，这 8 个函数最值得反复看。

## 12. 下一轮建议怎么继续精读

如果你接下来想“一模块一模块地啃”，我建议直接按下面顺序继续：

1. `FastExplorationFSM::FSMCallback()`
2. `FastExplorationManager::planExploreMotion()`
3. `FrontierFinder::searchFrontiers()`
4. `FrontierFinder::computeFrontiersToVisit()`
5. `FrontierFinder::updateFrontierCostMatrix()`
6. `FastExplorationManager::findGlobalTour()`
7. `FastExplorationManager::refineLocalTour()`
8. `FastPlannerManager::planExploreTraj()`
9. `FastPlannerManager::kinodynamicReplan()`
10. `FastPlannerManager::planYawExplore()`

这条顺序基本就是：

- 先搞清决策闭环
- 再看 frontier 数据结构
- 最后下钻轨迹与控制接口

