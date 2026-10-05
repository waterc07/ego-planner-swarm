# EGO 单机规划器（Boom_Birds fork）

基于 [FAST-Lab ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 的 ROS 2 移植，本项目使用 ROS 2 Jazzy。子模块目录名保留 `ego-planner-swarm`，源码只保留单机规划。

## 项目入口

`plan_manage/launch/boom_birds_offline.launch.py` 启动 `ego_planner_node` 和 `traj_server`。完整合成双目、深度、MAVROS、PX4 SIH 链路由母仓库 `boom_birds_sim` 启动，命令见 [工作空间 README](../../README.md) 和 [仿真运行说明](../boom_birds_nav/README.md)。

输入：

- `/boom_birds/depth/image`：米制 `32FC1`，无效深度为 NaN。
- `/boom_birds/depth/camera_info`：与图像时间、尺寸和光学帧匹配的内参。
- `/boom_birds/vio/camera_pose`：相机世界位姿。
- `/boom_birds/vio/odom_ego`：世界系位置与速度。

输出：`/boom_birds/ego/position_cmd`。项目会话模式经 `SessionBspline` 和 `PlannerCommand` 传递轨迹、任务与进程身份；取消先作废缓存，再确认屏障。MAVROS/PX4 通信由母仓库控制包负责。

## 单机代码

| 路径（相对 `src/planner/`） | 职责 |
| --- | --- |
| `plan_env` | 深度融合、占据地图、相机几何与断流检查 |
| `path_searching` | A* 局部搜索 |
| `bspline_opt` | 障碍物代价、平滑、动力学约束与候选轨迹优化 |
| `plan_manage` | 目标、重规划、紧急停止及轨迹执行 |
| `traj_utils` | 单条 Bspline、项目会话消息、轨迹容器与可视化 |

状态流为 `INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ`；执行中可进入 `REPLAN_TRAJ` 或 `EMERGENCY_STOP`。不等待其他无人机启动或轨迹。

已删除轨迹广播/接收、机间避碰代价、顺序启动、`drone_id`、`MultiBsplines`、`drone_detect`、`rosmsg_tcp_bridge`、`multi_map_server` 和多机 launch/RViz 显示。`use_distinctive_trajs` 保留：它生成同一架无人机的多条绕障候选。

`Bspline` 不再含 `drone_id`，与旧消息定义不兼容。修改后须在新安装前缀重建 `traj_utils`、优化器、规划器及使用这些消息的项目包；不要混用旧安装目录。

`src/uav_simulator` 中的单机仿真工具保留。本项目验收使用合成双目 → 深度 → EGO → MAVROS → PX4 SIH；原生仿真不是这条链路的验收入口。

## 验证

构建、脱机回归和 SIH 结果记录在母仓库 [STATUS](../../../../docs/STATUS.md)。真机、真实 VIO 和 Pi 5 结果单独记录，不能由 SIH 代替。

上游版权与许可证见 [LICENSE](LICENSE)。
