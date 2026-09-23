"""Boom_Birds 脱机接入启动（EGO 侧）。

把 ego_planner 节点接到 Boom_Birds 脱机链路的话题上，只启用一条地图输入路径
（深度图 + 相机 PoseStamped，grid_map pose_type=1）。

关键设置与依据（见 plan_env/src/grid_map.cpp 与 boom_birds_nav/config/contract.yaml）：

- pose_type=1：深度回调直接把收到的 PoseStamped 当相机位姿；cam2body_ 只服务 pose_type=2，
  所以相机位姿必须由 boom_birds_nav 的 pose_adapter 按 T_W_Crect 算好后发布。
- use_depth_filter=false：走无过滤投影分支（该分支已按 [BB-PATCH-1/2/3] 跳过无效观测）。
- k_depth_scaling_factor=1000：与官方 pcl_render_node 一致；上游发布米制 32FC1，
  转换后为毫米 uint16，再由同一因子还原为米。
- skip_pixel=1：该分支按列索引 u 取深度，skip_pixel>1 的采样语义见 plan_env 补丁说明。
- 参数类型：数值/布尔参数用 ParameterValue(value_type=...) 传入。rclcpp 的
  declare_parameter 会拒绝与声明类型不一致的覆盖值（例如 skip_pixel 声明为 int，
  传字符串会抛 InvalidParameterTypeException 导致节点起不来）。
- 相机内参与 /boom_birds/depth/camera_info 同源（由标定 P1 推导，320×240）：
  fx=fy=172.5980149526744、cx=156.8623504638672、cy=116.15113067626953。
  可用 launch 参数覆盖，但必须与深度节点实际使用的标定一致。
- odom_world 使用 /boom_birds/vio/odom_ego（twist=世界系速度，符合 EGO 回调预期），
  不是标准机体里程计 /boom_birds/vio/odom_body。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id")
    cx = ParameterValue(LaunchConfiguration("cx"), value_type=float)
    cy = ParameterValue(LaunchConfiguration("cy"), value_type=float)
    fx = ParameterValue(LaunchConfiguration("fx"), value_type=float)
    fy = ParameterValue(LaunchConfiguration("fy"), value_type=float)
    map_size_x = ParameterValue(LaunchConfiguration("map_size_x"), value_type=float)
    map_size_y = ParameterValue(LaunchConfiguration("map_size_y"), value_type=float)
    map_size_z = ParameterValue(LaunchConfiguration("map_size_z"), value_type=float)
    goal_x = ParameterValue(LaunchConfiguration("goal_x"), value_type=float)
    goal_y = ParameterValue(LaunchConfiguration("goal_y"), value_type=float)
    goal_z = ParameterValue(LaunchConfiguration("goal_z"), value_type=float)
    max_vel = ParameterValue(LaunchConfiguration("max_vel"), value_type=float)
    max_acc = ParameterValue(LaunchConfiguration("max_acc"), value_type=float)
    resolution = ParameterValue(LaunchConfiguration("resolution"), value_type=float)
    obstacles_inflation = ParameterValue(LaunchConfiguration("obstacles_inflation"), value_type=float)
    max_ray_length = ParameterValue(LaunchConfiguration("max_ray_length"), value_type=float)

    args = [
        DeclareLaunchArgument("ready_min_fusion_updates", default_value="5"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("map_size_x", default_value="40.0"),
        DeclareLaunchArgument("map_size_y", default_value="40.0"),
        DeclareLaunchArgument("map_size_z", default_value="6.0"),
        DeclareLaunchArgument("cx", default_value="156.8623504638672"),
        DeclareLaunchArgument("cy", default_value="116.15113067626953"),
        DeclareLaunchArgument("fx", default_value="172.5980149526744"),
        DeclareLaunchArgument("fy", default_value="172.5980149526744"),
        DeclareLaunchArgument("max_vel", default_value="1.0"),
        DeclareLaunchArgument("max_acc", default_value="1.0"),
        DeclareLaunchArgument("goal_x", default_value="2.5"),
        DeclareLaunchArgument("goal_y", default_value="1.2"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),
        DeclareLaunchArgument("obstacles_inflation", default_value="0.15"),
        DeclareLaunchArgument("resolution", default_value="0.1"),
        DeclareLaunchArgument("max_ray_length", default_value="4.5"),
    ]

    planner = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name=["drone_", drone_id, "_ego_planner_node"],
        output="screen",
        remappings=[
            ("odom_world", "/boom_birds/vio/odom_ego"),
            ("grid_map/depth", "/boom_birds/depth/image"),
            ("grid_map/pose", "/boom_birds/vio/camera_pose"),
            # 不启用点云地图路径：话题保留且无发布者，避免误接第二条地图输入
            ("grid_map/cloud", "/boom_birds/depth/xyz_unused"),
            ("grid_map/odom", "/boom_birds/vio/odom_ego"),
            ("grid_map/occupancy", "/boom_birds/ego/grid_map/occupancy"),
            ("grid_map/occupancy_inflate", "/boom_birds/ego/grid_map/occupancy_inflate"),
            ("planning/bspline", "/boom_birds/ego/planning/bspline"),
            ("planning/data_display", "/boom_birds/ego/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/boom_birds/ego/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/boom_birds/ego/broadcast_bspline"),
        ],
        parameters=[
            {"fsm/flight_type": 2},
            {"fsm/thresh_replan_time": 1.0},
            {"fsm/thresh_no_replan_meter": 1.0},
            {"fsm/planning_horizon": 7.5},
            {"fsm/planning_horizen_time": 3.0},
            {"fsm/emergency_time": 1.0},
            {"fsm/realworld_experiment": False},
            {"fsm/fail_safe": True},
            {"fsm/waypoint_num": 1},
            {"fsm/waypoint0_x": goal_x},
            {"fsm/waypoint0_y": goal_y},
            {"fsm/waypoint0_z": goal_z},
            {"grid_map/ready_min_fusion_updates": ParameterValue(LaunchConfiguration("ready_min_fusion_updates"), value_type=int)},
            {"grid_map/resolution": resolution},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": 5.5},
            {"grid_map/local_update_range_y": 5.5},
            {"grid_map/local_update_range_z": 4.5},
            {"grid_map/obstacles_inflation": obstacles_inflation},
            {"grid_map/local_map_margin": 10},
            {"grid_map/ground_height": -0.01},
            {"grid_map/cx": cx},
            {"grid_map/cy": cy},
            {"grid_map/fx": fx},
            {"grid_map/fy": fy},
            {"grid_map/use_depth_filter": False},
            {"grid_map/depth_filter_tolerance": 0.15},
            {"grid_map/depth_filter_maxdist": 5.0},
            {"grid_map/depth_filter_mindist": 0.2},
            {"grid_map/depth_filter_margin": 2},
            {"grid_map/k_depth_scaling_factor": 1000.0},
            {"grid_map/skip_pixel": 1},
            {"grid_map/p_hit": 0.65},
            {"grid_map/p_miss": 0.35},
            {"grid_map/p_min": 0.12},
            {"grid_map/p_max": 0.90},
            {"grid_map/p_occ": 0.80},
            {"grid_map/min_ray_length": 0.1},
            {"grid_map/max_ray_length": max_ray_length},
            {"grid_map/virtual_ceil_height": 2.9},
            {"grid_map/visualization_truncate_height": 1.8},
            {"grid_map/show_occ_time": False},
            {"grid_map/pose_type": 1},
            {"grid_map/frame_id": "global"},
            {"grid_map/odom_depth_timeout": 1.0},
            {"manager/max_vel": max_vel},
            {"manager/max_acc": max_acc},
            {"manager/max_jerk": 3.0},
            {"manager/control_points_distance": 0.4},
            {"manager/feasibility_tolerance": 0.05},
            {"manager/planning_horizon": 7.5},
            {"manager/use_distinctive_trajs": True},
            {"manager/drone_id": 0},
            {"optimization/full_path_collision_check": True},
            {"optimization/lambda_smooth": 1.0},
            {"optimization/lambda_collision": 0.5},
            {"optimization/lambda_feasibility": 0.1},
            {"optimization/lambda_fitness": 1.0},
            {"optimization/dist0": 0.5},
            {"optimization/swarm_clearance": 0.5},
            {"optimization/max_vel": max_vel},
            {"optimization/max_acc": max_acc},
            {"bspline/limit_vel": max_vel},
            {"bspline/limit_acc": max_acc},
            {"bspline/limit_ratio": 1.1},
            {"prediction/obj_num": 0},
            {"prediction/lambda": 1.0},
            {"prediction/predict_rate": 1.0},
        ],
    )

    traj_server = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server",
        output="screen",
        remappings=[
            ("planning/bspline", "/boom_birds/ego/planning/bspline"),
            ("/position_cmd", "/boom_birds/ego/position_cmd"),
        ],
        parameters=[{"traj_server/time_forward": 1.0}],
    )

    return LaunchDescription(args + [planner, traj_server])
