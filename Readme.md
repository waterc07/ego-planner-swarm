# EGO-Planner-Swarm（ROS 2 Humble 版本）

[EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 的 ROS 2 移植版本。EGO-Swarm 是浙大 FAST-Lab 提出的**去中心化无人机集群自主导航系统**（ICRA 2021），核心目标：

> 让多架四旋翼仅凭机载传感器（深度相机 / 点云），在未知的密集障碍环境里，去中心化、异步地各自规划并协同避让，飞到各自目标点。

**没有中央调度器**——每架无人机独立建图、独立规划、独立避障，只通过广播自己的 B 样条轨迹来"告知"邻居。这是它区别于集中式集群方案的本质。

其核心算法 EGO-Planner 是 **ESDF-free** 的（论文标题即 *EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors*）：不维护欧氏距离场，而用 A\* 路径几何构造排斥方向，省掉距离场的维护开销。

---

# 使用方法
## 1. 需要的库 
* vtk(是安装PCL的依赖库，编译时需要勾选Qt)
* PCL

## 2. 前置条件
可能是我一些发布订阅的设置写的不太对，使用ROS2默认的FastDDS会导致程序运行很卡，目前还没找到原因，所以请按照下述方法将DDS修改为cyclonedds

### 2.1 安装cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 修改默认的DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 检查是否修改成功
```
ros2 doctor --report | grep "RMW middleware"
```
输出显示rmw_cyclonedds_cpp则说明修改成功

## 3. 代码运行
### 3.1 运行Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 运行规划程序
新开一个终端，输入以下指令
* 单机
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* 附加参数，可以选择地图生成模式以及是否考虑动力学
    * use_mockamap:地图生成方式，默认为False，False时使用Random Forest, True时使用mockamap
    * use_dynamic:是否考虑动力学，默认为False, False时不考虑, True时考虑
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

---

# 项目架构

## 4. 分层总览

当前编译配置下的启动规模：

| launch 文件 | 无人机数 | 地图尺寸 (m) |
|---|---|---|
| `single_run_in_sim.launch.py` | 1 | 42 × 30 × 5 |
| `swarm.launch.py` | 10 | 42 × 30 × 5 |
| `swarm_large.launch.py` | 21 | 72 × 30 × 5 |

集群场景是**对穿飞行**——各机从 y 轴一侧一字排开飞向对面，中途必然交叉，专门用来验证协同避让。

整个工作空间共 **20 个 ROS 2 包**，按职责分成六层：

```
┌─────────────────────────────────────────────────────────┐
│ L5 交互/可视化   waypoint_generator · odom_visualization  │
├─────────────────────────────────────────────────────────┤
│ L4 控制层        traj_server → poscmd_2_odom（运动学）     │
│                            └→ so3_control → SO3 动力学   │
├─────────────────────────────────────────────────────────┤
│ L3 集群协同      broadcast_bspline 广播/接收 · bridge_node │
├─────────────────────────────────────────────────────────┤
│ L2 规划层        plan_manage(FSM) ─ bspline_opt(后端优化)  │
│                                     └ path_searching(前端) │
├─────────────────────────────────────────────────────────┤
│ L1 感知/建图     plan_env(GridMap) · obj_predictor         │
├─────────────────────────────────────────────────────────┤
│ L0 仿真/世界     local_sensing(虚拟深度相机) · map_generator│
│                  / mockamap · so3_quadrotor_simulator      │
└─────────────────────────────────────────────────────────┘
```

代码量分布（C++）：

| 包 | 行数 | 角色 |
|---|---|---|
| `uav_simulator/local_sensing` | 2387 | 传感器仿真 |
| `planner/bspline_opt` | 2332 | **算法核心**：B 样条优化 |
| `planner/plan_env` | 1962 | 概率栅格地图 |
| `planner/plan_manage` | 1890 | 状态机 + 调度 |
| `uav_simulator/mockamap` | 1464 | 程序化地图生成 |
| `planner/rosmsg_tcp_bridge` | 866 | 真机 UDP 通信 |

---

## 5. 各层详解

### L0 仿真 / 世界层 —— 制造一个"未知世界"

`simulator.launch.py` 里挂三类节点：

**地图生成**（二选一，由 `use_mockamap` 决定）

| 包 | 内容 |
|---|---|
| `map_generator` | 随机方块 + 圆柱障碍，`min_distance` 保证最小间距 |
| `mockamap` | 4 类程序化地图：Perlin 噪声 / 随机长方体 / 递归分割迷宫 / Voronoi 三维迷宫 |

两者都发布 `/map_generator/global_cloud`——**这是真值地图，规划器不许看**。

**传感器仿真** `local_sensing`：`pcl_render_node` 订阅真值地图 + 里程计，模拟一个带内参的传感器，输出 `pcl_render_node/cloud`。**规划器只能看到这个**。

> ⚠️ 详见 [第 8 节](#8-已知问题与代码现状)——默认配置下这个传感器**没有遮挡判断**。

**动力学仿真** `so3_quadrotor_simulator`：牛顿-欧拉刚体方程 + 一阶电机模型，1 kHz RK4 积分。

**两种"无人机"模式**（`use_dynamic` 参数）：

| 模式 | 节点 | 行为 |
|---|---|---|
| `false`（默认） | `poscmd_2_odom` | 把位置指令**直接抄成里程计**，跳过动力学。省算力，可支撑几十架 |
| `true` | `so3_control` + 动力学 | 真实但有计算开销 |

### L1 感知 / 建图层 —— 把点云变成可查询的几何

核心是 `plan_env/grid_map.cpp`：一个**概率占据栅格**，双缓冲：

```cpp
std::vector<double> occupancy_buffer_;         // log-odds 概率值
std::vector<char>   occupancy_buffer_inflate_; // 膨胀后的 0/1 栅格 ← 规划器只查这个
```

更新是标准二值贝叶斯，在 log-odds 空间累加后 clamp：

```cpp
log_odds_update = (count_hit >= count_miss) ? prob_hit_log_ : prob_miss_log_;   // ±0.619
occupancy_buffer_[idx] = clamp(occupancy_buffer_[idx] + log_odds_update,
                               clamp_min_log_, clamp_max_log_);                 // [logit(0.12), logit(0.90)]
```

**两条互不干扰的更新通道**：

| 通道 | 触发 | 概率更新 | 说明 |
|---|---|---|---|
| A 深度图 raycast | `depth` + 位姿同步 | ✅ | 主通道，模拟真实相机。用 Amanatides & Woo DDA 体素遍历 |
| B 直接点云 | `grid_map/cloud` | ❌ | 直接写膨胀栅格，仿真环境用 |

关键参数：`resolution=0.1m`、`p_hit=0.65`/`p_miss=0.35`、`p_occ=0.80`、`obstacles_inflation=0.099`（≈1 格）、`virtual_ceil_height=2.9m`、42×30×5 地图 ≈ **63 万体素**。

> **膨胀半径只有 1 个栅格**，远小于无人机尺寸。安全距离**不靠地图膨胀保证**，而靠优化器里的 `dist0=0.5` / `swarm_clearance=0.5`。地图膨胀在这里只是为了让 A\* 不会搜出贴着障碍表面的路径。

查询接口全部内联、**O(1) 查表、无插值**：`getInflateOccupancy` / `getOccupancy` 就是「越界检查 → `posToIndex` → 一维数组取值」。

**没有 ESDF。** 全仓库 `evaluateEDT` / `getDistance` / `getSurroundPts` / `sdf_map` 均为 0 处命中——距离代价改由几何方法构造，见 [7.2](#72-后端bspline_opt)。

动态障碍：`obj_predictor` 用**常速度外推**（取历史队列最后两点解 2×2 线性系统）预测移动物体位置。代码里还有正经的 5 阶多项式最小二乘版本，但被注释掉了。

### L2 规划层 —— 前端 + 后端两段式

**这是整个项目的核心。**

**前端 `path_searching`**：只有一个文件 `dyn_a_star.cpp`（类名仍叫 `AStar`）。

| 实现要点 | 值 |
|---|---|
| 邻域 | 26 邻域扩展 |
| 启发函数 | 对角距离 `√3·diag + √2·min + 1·rest`，**加权** `tie_breaker = 1 + 1e-4` |
| 节点池 | 硬编码 100³（`planner_manager.cpp:43`） |
| 步长 / 超时 | `step_size=0.1` / 0.2s |
| 与地图的耦合 | 仅一行：`getInflateOccupancy(pos)` |

用 `rounds_` 轮次号代替每次搜索清空全图，避免百万节点的重置开销。

> **注意**：A\* 不是独立规划节点，而是 `BsplineOptimizer` 的成员，在 `initControlPoints`（初始化）和 `check_collision_and_rebound`（优化中途）**两处都会调用**。它是 rebound 机制的核心部件。

原版 EGO-Planner 的 `astar.cpp`、`kinodynamic_astar.cpp`、`topo_prm.cpp` 在本仓库**均不存在**。

**后端 `bspline_opt`**：把轨迹表示成 B 样条（控制点即优化变量），用 **L-BFGS** 求解。

L-BFGS 配置：

```cpp
lbfgs_params.mem_size       = 16;     // 默认 8
lbfgs_params.max_iterations = 200;
lbfgs_params.g_epsilon      = 0.01;   // rebound；refine 用 0.001
```

优化变量切片（决定哪些控制点能动）：

| 阶段 | 固定 | 自由 |
|---|---|---|
| `rebound_optimize` | 前 3 个 | 其余全部（末端自由） |
| `refine_optimize` | 前 3 个 **+ 后 3 个** | 中间全部 |

前 3 个固定是为了保证与当前飞行状态的**速度/加速度连续**，否则每次重规划轨迹都会在起点跳变。

**调度 `plan_manage`**：`EGOPlannerManager` 把前端后端串起来。

`reboundReplan` 三步流程：

```
STEP 1  INIT      多项式 / 随机多项式 / 沿用当前轨迹 / A* 初始化 → parameterizeToBspline
STEP 2  OPTIMIZE  distinctiveTrajs 生成最多 8 条候选 → 逐条 L-BFGS → 取 final_cost 最小
STEP 3  REFINE    时间重分配（仅 drone_id ≤ 0 启用）
```

**STEP 1 的四种初始化策略按条件切换**：

| 策略 | 触发条件 |
|---|---|
| 多项式（min-snap 单段） | 首飞 / 新目标 |
| **随机多项式** | 同一状态连续失败后，在起终点中点加随机偏移。**随机幅度随失败次数递减**，从"大胆试探"退化为"保守微调" |
| 沿用当前轨迹 | 常规重规划，从 `t_cur` 采样剩余部分 |
| A\* 初始化 | 在优化器内部 `initControlPoints` 完成 |

**STEP 2 的失败重试有明确升级路径**，收敛后三阶段检查：

1. `min_ellip_dist_ > swarm_clearance_`？蜂群太挤 → 重启，且 **`new_lambda2_ *= 2`**（加倍避障权重）
2. 前 2/3 有碰撞？**若前 3 个控制点就在障碍里 → 直接判失败**（起点本身不可行，救不回来）；否则用 A\* 重新反弹再优化
3. 最多重启 **3 次**

**STEP 3 在集群模式下被禁用**（`planner_manager.cpp:297`）：

```cpp
if (pp_.drone_id <= 0)   // 单机 或 drone_0
{ ... refineTrajAlgo(...) ... }
else
{ RCLCPP_ERROR(..., "IN SWARM MODE, REFINE DISABLED!"); }
```

原因很直接：refine 会拉伸轨迹时间轴，而集群避让全靠各机**在同一全局时间轴上比对位置**——任何一架私自改时间尺度，邻居的预测就全错了。所以 swarm 里 `max_vel`/`max_acc` 是硬约束，超了就重规划，不改时间。

**有限状态机** `ego_replan_fsm.cpp`，7 个状态：

```cpp
enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ,
                      EXEC_TRAJ, EMERGENCY_STOP, SEQUENTIAL_START };
enum TARGET_TYPE    { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };
```

由**双定时器**驱动（不是事件驱动）：

| 定时器 | 周期 | 回调 |
|---|---|---|
| `exec_timer_` | **10ms (100Hz)** | `execFSMCallback` — 主状态机，开头 `cancel()` 防重入、结尾 `reset()` |
| `safety_timer_` | **50ms (20Hz)** | `checkCollisionCallback` — 独立安全兜底 |

```
INIT ──have_odom_──→ WAIT_TARGET ──have_target_ && have_trigger_──→ SEQUENTIAL_START
                                                                          │
                          ┌───────────────────────────────────────────────┘
                          ↓ (首条轨迹成功)
                    EXEC_TRAJ ←──── REPLAN_TRAJ
                       │  ↑              ↑
                       │  └── 补救成功 ───┤
                       ↓                 │ t_cur > replan_thresh_
                  EMERGENCY_STOP ────────┘
                       │
                       └─ enable_fail_safe_ && ‖v‖<0.1 ──→ GEN_NEW_TRAJ
```

**SEQUENTIAL_START** 是集群专属：drone 0 直接起飞；drone *i*≥1 必须等到 `have_recv_pre_agent_`（收到 drone *i-1* 的轨迹链）才规划——**顺序起飞**，避免所有机同时解锁互相撞。

**EXEC_TRAJ** 做三件事：推进航点（距当前路点 < `no_replan_thresh_`=1.0m）、判断到达终点、定时重规划（`t_cur > replan_thresh_`=1.0s）。

### L3 集群协同层 —— 去中心化怎么实现

**两条完全独立的轨迹交换链路**，这是 EGO-Swarm 设计上最精巧的地方：

**(a) 顺序启动链：`MultiBsplines`，单链传递**

```
drone0 ──swarm_trajs──→ drone1 ──swarm_trajs──→ drone2 ──→ ...
```

每架订阅**前一架**的 `/drone_{i-1}_planning/swarm_trajs`。发布时把**自己收到的整条链**追加自己的轨迹后转发——于是 drone *k* 发出的消息含 0..*k* 共 *k+1* 条轨迹。

这是 **O(n) 的链式累积**，而非 O(n²) 全互联。代价是只在启动时用一次，且**任意一环断了后面全起不来**。

**(b) 实时避碰：`Bspline`，公共广播**

每机把每次重规划结果发到公共 topic `/broadcast_bspline`，所有机都订阅同一个。收到后：

1. 忽略自己
2. **时间戳偏差 > 0.25s 直接丢弃**——时钟不同步就没法比
3. 距离过滤：用前三个控制点估出对方起点，距离 > `planning_horizon × 4/3` 就不入库
4. 存入 `swarm_trajs_buf_[id]`
5. **`if (checkCollision(id)) changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK")`** ← 立刻触发重规划

第 5 步是关键：不是等自己的定时器到点，而是**一收到冲突轨迹就马上重规划**。

配套的安全层 `checkCollisionCallback`（20Hz）：沿当前轨迹**前 2/3** 以 0.01s 步长查膨胀栅格 + 与其他机距离 < 1.0m。发现碰撞先试补救；失败则看碰撞是否发生在 `emergency_time_`(1.0s) 内——是则急停，否则重规划。

> 为什么只查前 2/3？因为后 1/3 反正马上会被下一次重规划覆盖掉，查了是浪费。

**真机部署** `rosmsg_tcp_bridge`：不走 DDS，改用 **UDP 广播(8081)** 手动序列化轨迹和里程计，另开 TCP(8080) 接收地面站指令。原因很实际——几十架无人机的 DDS 服务发现会压垮 WiFi 自组网。

**`drone_id` 参与的所有决策**：是否订阅上一机轨迹、发布 topic 名、SEQUENTIAL_START 是否等待、refine 是否启用、swarm 代价中排除自己。约定 `<= -1` 为单机、`>= 0` 为集群。

### L4 控制层 —— 轨迹变指令

**`traj_server`**：把收到的 `Bspline` 消息还原成 `UniformBspline` 对象，以 100Hz 采样出 `PositionCommand`（位置/速度/加速度/yaw），且**向前看 `time_forward=1.0s`** 提前起飞。yaw 按**速度方向**算，限幅 π rad/s 并做低通滤波。

**动力学模式下，位置环和姿态环是分开的**：

| 环节 | 位置 | 公式 |
|---|---|---|
| 位置环 | `so3_control/SO3Control.cpp` | `F = mg·e₃ + kx·ep + kv·ev + m·ka·ea + m·a_des` |
| 姿态环 | `so3_quadrotor_simulator`（仿真器内部） | `M = -kR·eR - kOm·eOm + ω×Jω` |

`so3_control` 发 `SO3Command`（力 + 期望姿态 + kR/kOm 增益），**增益随消息下发**，真正算力矩的是仿真器。想换姿态控制器，要改仿真器而不是 `so3_control`。

位置环还有两个细节：`ka = 0.2·|err|` 的自适应加速度增益（误差 >3 时归零），以及 **45° 倾斜限幅**。

### L5 可视化 / 交互

`waypoint_generator`（RViz 点击目标）、`odom_visualization`（机体 mesh、轨迹、速度矢量、TF 广播）。

`multi_map_server`（多机地图融合的栅格类与消息）**当前规划链路完全没有用到**，是历史遗留。

---

## 6. 核心机制详解

### 6.1 数据流

```
map_generator ─┐
               ├─→ /map_generator/global_cloud
mockamap ──────┘         │
                         ↓
                 pcl_render_node ──→ drone_N_pcl_render_node/cloud ──┐
                   (↑ odometry)                    + depth (仅CUDA)  │
                                                                    ↓
                                                       GridMap（概率栅格 + 膨胀）
                                                                    │
EGOReplanFSM → planning/bspline → traj_server → drone_N_planning/pos_cmd
                                                                    │
                        ┌───────────────────────────────────────────┤
                        ↓ use_dynamic=true                use_dynamic=false
                  so3_control → so3_cmd                    poscmd_2_odom
                        ↓                                        │
                so3_quadrotor_simulator (1kHz RK4)               │
                        ↓                                        │
                  drone_N_visual_slam/odom ←─────────────────────┘
                        │
        ┌───────────────┼────────────────┐
        ↓               ↓                ↓
   EGOReplanFSM    GridMap         odom_visualization
   (odom_world)    (grid_map/odom)  (RViz Marker/TF)
```

### 6.2 后端：bspline_opt

**Rebound 相**（`combineCostRebound`）的代价合成：

```
f = λ1·平滑度 + new_λ2·(障碍距离 + 集群间距) + λ3·动力学可行性 + λ2·终端
```

**Refine 相**（`combineCostRefine`）：

```
f = λ1·平滑度 + λ4·拟合 + λ3·动力学可行性
```

各代价项：

| 项 | 公式 / 思路 |
|---|---|
| **平滑 (jerk)** | 三次 B 样条的 jerk 正比三阶差分：`J_i = Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i`，`f = Σ‖J_i‖²`，梯度 `2J_i·[-1,+3,-3,+1]` |
| **距离 (ESDF-free)** | `dist = (Q_i − base_point)·direction`，`dist_err = clearance − dist`。**分段**：`dist_err < 0` 无代价；`< demarcation` 用三次式 `dist_err³`；超出用二次式。三次到二次的 C¹ 延拓 |
| **动力学可行性** | `\|v_i\| > max_vel → (v_i ∓ max_vel)²·(1/ts²)`；`\|a_i\| > max_acc → (a_i ∓ max_acc)²`。速度项乘 `1/ts²` 使量级一致 |
| **集群 (椭球)** | 椭球距离 `sqrt(dz²/a² + (dx²+dy²)/b²)`，**a=2.0 (z 半轴)、b=1.0 (xy 半轴)**（垂直容忍度大一倍），`CLEARANCE = swarm_clearance × 2` |
| **终端** | `dq = (Q_{N-3} + 4Q_{N-2} + Q_{N-1})/6 − local_target_pt_`，`cost = ‖dq‖²`。权重用 `lambda2_` 而非 `new_lambda2_`——**终点约束不参与避障权重加倍**，无论撞多少次终点都必须贴合 |
| **拟合 (仅 refine)** | `f = (x·v)²/25 + ‖x×v‖²/1`，x 为偏离量、v 为局部切向。**切向容差是法向的 25 倍**——允许沿轨迹方向偏离，但横向惩罚重，用于保拓扑 |

**距离代价怎么在没有 ESDF 的情况下算出方向？** 三步几何构造（`check_collision_and_rebound`）：

1. 检测到轨迹段与障碍碰撞后，对该段起终点跑一次 A\*，得到绕障路径
2. 求 **A\* 路径与轨迹控制点连线的交点**
3. 从控制点朝交点方向**以分辨率 0.1m 为步长扫描**，找到第一个占据体素记为 `base_point`，方向记为单位向量 `direction`

代价触发有条件：`calcDistanceCostRebound` 只在 `iter_num > 3 && smoothness_cost/(cps_.size−2·order_) < 0.1` 时才调 `check_collision_and_rebound()`——**先让轨迹变平滑，再处理碰撞**（0.1 是注释里标明的实验值）。

**"弹性带"（elastic band）机制**：

```
优化中发现新碰撞 → check_collision_and_rebound() 置 force_stop_type_ = STOP_FOR_REBOUND
                ↓
earlyExit 回调返回非零 → L-BFGS 返回 LBFGSERR_CANCELED
                ↓
rebound_times++，用**扩充后的** base_point/direction 集合重新优化（最多 20 次）
```

关键在**每次回弹往 `cps_.base_point`/`cps_.direction` 里 push 新的排斥向量对，而不是替换旧的**。障碍约束像橡皮筋一样一条条挂上去、越拉越紧——这就是这个名字的字面含义。

**`distinctiveTrajs` 为什么是 8 条**：

```cpp
constexpr int MAX_TRAJS = 8;   // 最多的轨迹数量
constexpr int VARIS     = 2;   // 允许的变化种类数
int seg_upbound = min(segments.size(), floor(log(MAX_TRAJS)/log(VARIS)));  // = min(n, 3)
```

每条碰撞段只有两种绕法（左绕 / 右绕），最多挑 **3 段**做变换 → **2³ = 8** 条拓扑互异的候选轨迹。如果碰撞段更多，也只变前 3 段，其余沿用 A\* 的反弹结果。

### 6.3 UniformBspline 与时间重分配

| 接口 | 用在哪 |
|---|---|
| `parameterizeToBspline(ts, point_set, deriv, ctrl_pts)` | 采样点 → 控制点（**静态方法**，QR 求解，K+4 行方程） |
| `evaluateDeBoor(u)` | de Boor 递推求值 |
| `getDerivativeControlPoints()` | 由位置控制点推速度/加速度控制点（B 样条的导数仍是 B 样条） |
| `checkFeasibility(ratio)` | 判断是否超限，`ratio` 输出超限程度 |
| `lengthenTime(ratio)` | **时间重分配的核心** |
| `getLength()` / `getJerk()` | 轨迹质量度量 |

B 样条之所以是这套系统的核心选择，关键在两点：

- `getDerivativeControlPoints()` 让速度/加速度约束**直接在控制点上表达**，不必对轨迹做数值微分——这是高效求梯度的前提
- `lengthenTime()` 让时间重分配变成**纯代数操作**（拉伸节点向量）

**时间重分配不是 `UniformBspline` 单独完成的**，类里并没有 `reallocateTime`。真正的流程是 `planner_manager::reparamBspline` 里的三步组合：

```
lengthenTime(ratio)         // ← 只拉伸"中段"！节点 5..K-5 线性分摊额外时间，尾部整体平移
    ↓
按新 dt 重采样 K 个点
    ↓
parameterizeToBspline(...)  // QR 求解新控制点
    ↓
BsplineOptimizeTrajRefine
```

`lengthenTime` **只拉伸中段、保持起止时刻**——起止时刻不变才能与集群其他机的时间轴对齐。

### 6.4 GridMap 的实现细节

**raycast 用 Amanatides & Woo DDA 体素遍历**（1987）：用 `tMaxX/Y/Z`（穿过体素边界的参数 t）与 `tDelta`，每步只比较三个轴、走参数最小的轴，**不产生空洞、每体素 O(1)**。

**去重**靠 `flag_traverse_` / `flag_rayend_` 存 `raycast_num_` 帧号——每帧每体素只计一次。

**通道 A 的三步**：`projectDepthImage`（按 `skip_pixel=2` 抽稀，针孔模型反投影）→ `raycastProcess`（沿途写 miss、终点写 hit）→ `clearAndInflateLocalMap`（范围外清 unknown + 膨胀）。

**虚拟天花板** `virtual_ceil_height=2.9m`：直接往膨胀栅格顶部糊一整层占据，用地图机制硬性限高。

### 6.5 动力学仿真

**微分平坦只用在规划端**（轨迹用多项式/B 样条表示，位置即平坦输出）。仿真端是老老实实的刚体方程：

**状态 22 维**：`x(3) + v(3) + R(3×3) + ω(3) + motor_rpm(4)`

```
ẋ = v
v̇ = -g·e₃ + T·R·e₃/m + F_ext/m - 阻力·v̂/m       T = kf·Σrpm²
Ṙ = R·ω̂
ω̇ = J⁻¹(τ - ω×Jω + M_ext)                        τ 由转速差动产生
rpṁ = (rpm_cmd - rpm)/τ_motor                    τ = 1/30 s 一阶电机
```

参数：`m = 0.98 kg`、`J = diag(2.64e-3, 2.64e-3, 4.96e-3)`、`kf = 8.98132e-9`、转速 ∈ [1200, 35000]，二次阻力 C=0.1。

**积分**：Boost odeint `runge_kutta4` 固定步长，节点 1000Hz → **1 kHz RK4**。每步后做 R 的极分解正交化（防数值漂移出 SO(3)）、地面碰撞处理（z<0）、NaN 回滚。

**混控**为 X 型布局：1 前 2 后 3 右 4 左。

---

## 7. 已知问题与代码现状

以下是阅读源码时发现的、**当前版本的实际情况**，按影响程度排列：

| 问题 | 位置 | 影响 |
|---|---|---|
| **传感器无遮挡判断** | `local_sensing/src/pointcloud_render_node.cpp` | 默认 CPU 版只做「球形半径 + 视场角裁剪」（仰角 ≤30°、水平 ±60°），**没有 raycast 也没有深度缓冲**。只要障碍落在 4.5m 半径视锥内，即使被完全挡住也会出现在点云里——**无人机能"看穿"墙**。当前仿真验证的是"理想感知下的规划"，不是真实感知下的 |
| **refine 重试循环失效** | `bspline_optimizer.cpp` `refine_optimize` 结尾 | `do { ...; iter_count++; } while (!flag_safe && iter_count <= 0);` —— 第一轮后 `iter_count=1`，条件恒假，**循环只跑一次**。于是 `lambda4_ *= 2`（撞障加倍拟合权重）和紧随的恢复语句都不可达。refine 撞障时不重试，直接返回失败 |
| **地图外 = 障碍** | `plan_env/include/plan_env/grid_map.h` `getInflateOccupancy` | 越界返回 **-1**，而所有调用点都是布尔语义 `if (getInflateOccupancy(pt))`——**-1 在 C++ 里为真**。地图外的采样点被保守地判为占据。实践中靠"规划视界 << 地图尺寸"回避 |
| **动态障碍避让整体关闭** | `bspline_optimizer.cpp:1822` + `run_in_sim.launch.py` | `calcMovingObjCost` 被注释掉，`obj_generator` 节点未启用。生成器、预测器、代价项代码齐全但全未接入。启用需改两处 |
| **无 jerk 约束** | `bspline_optimizer.cpp` `calcFeasibilityCost` | `#define SECOND_DERIVATIVE_CONTINOUS` 被注释，生效的是简化二次罚分支。`max_jerk` 参数读进来了但**没用在任何代价里** |
| **集群禁用时间重分配** | `planner_manager.cpp:297` | 设计权衡而非 bug：时间轴对齐与独立调时间根本冲突。后果是 swarm 下 `max_vel`/`max_acc` 成为硬约束 |
| **大量 v1 死代码** | `bspline_optimizer.h`、`gradient_descent_optimizer.cpp` | 头文件声明的 `costFunction`、`combineCost`、`optimize()`、`setGuidePath`、`setWaypoints`、带 `cost_function` 参数的 `BsplineOptimizeTraj` **均无实现**；整个梯度下降求解器无人调用。`EGOReplanFSM::checkCollision()` 也是声明了没定义 |
| **`pubSensedPoints` 被短路** | `map_generator/src/random_forest_sensing.cpp:308` | 一个裸 `return;`（注释："有这个return后续的代码都不会执行"），局部地图发布代码是死的 |
| **无 BAG 枚举** | — | 上游 v1 的 `NORMAL_PHASE`/`REFINE_PHASE` 等**在本仓库不存在**，已硬编码成 `combineCostRebound` / `combineCostRefine` 两个函数。读代码时别被头文件的 v1 声明误导 |

**其他值得知道的点**：

- **CPU 频率影响可复现性**：规划耗时太短，OS 来不及升频会导致耗时抖动。建议 `sudo cpufreq-set -g performance`
- **集群通信靠"软"同步**：没有共识协议，靠 0.25s 时间窗 + 距离窗过滤。规模受通信距离和时钟同步精度限制
- **`planNextWaypoint` 在 FSM 里**（`ego_replan_fsm.cpp`），不在 `planner_manager.cpp`——名字容易误导
- **`execFSMCallback` 每周期发 `data_disp_`**，但只有 header 时间戳有值，其余字段为空

---

## 8. 分支说明

- `ros2_version`：保留原有 ROS 2 代码和 Humble 使用说明，补充中文架构文档与仓库忽略规则。
- `ros2_lyrical`：在原有 ROS 2 版本上增加 Lyrical 适配，包括 CMake 兼容包、头文件路径和订阅器 QoS API 修改。

本分支未引入 `ament_target_dependencies_compat`，也未应用上述 Lyrical API 修改。使用 Lyrical 时请切换到 `ros2_lyrical`，并按该分支的说明编译运行。

---

## 9. 参考文献

- **EGO-Swarm**: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments. Xin Zhou, Jiangchao Zhu, Hongyu Zhou, Chao Xu, Fei Gao. ICRA 2021. [Paper](https://ieeexplore.ieee.org/abstract/document/9561902)
- **EGO-Planner**: An ESDF-Free Gradient-Based Local Planner for Quadrotors. Xin Zhou, Zhepei Wang, Chao Xu, Fei Gao. RA-L 2020.
- 上游仓库：[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
