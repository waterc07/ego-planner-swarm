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
- use_camera_info=true：按图像时间、尺寸及光学帧核对 CameraInfo，禁止静态内参覆盖。
- use_camera_info=false：独立入口使用显式静态内参；不订阅 CameraInfo。
- odom_world 使用 /boom_birds/vio/odom_ego（twist=世界系速度，符合 EGO 回调预期），
  不是标准机体里程计 /boom_birds/vio/odom_body。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        DeclareLaunchArgument("world_frame", default_value="global"),
        DeclareLaunchArgument("local_target_search_radius_m", default_value="1.5"),
        DeclareLaunchArgument("local_target_search_step_m", default_value="0.2"),
        DeclareLaunchArgument("planning_horizon", default_value="7.5"),
        DeclareLaunchArgument("authorization_timeout_s", default_value="0.2"),
        DeclareLaunchArgument("command_future_tolerance_s", default_value="0.03"),
        DeclareLaunchArgument("depth_topic", default_value="/boom_birds/depth/image"),
        DeclareLaunchArgument("camera_info_topic", default_value="/boom_birds/depth/camera_info"),
        DeclareLaunchArgument("camera_pose_topic", default_value="/boom_birds/vio/camera_pose"),
        DeclareLaunchArgument("odom_topic", default_value="/boom_birds/vio/odom_ego"),
        DeclareLaunchArgument("planner_request_topic", default_value="/boom_birds/planner/request"),
        DeclareLaunchArgument("executor_status_topic", default_value="/boom_birds/planner/executor_status"),
        DeclareLaunchArgument("planner_status_topic", default_value="/boom_birds/planner/status"),
        DeclareLaunchArgument("planner_command_topic", default_value="/boom_birds/planner/command"),
        DeclareLaunchArgument("control_execution_topic", default_value="/boom_birds/control/execution_status"),
        DeclareLaunchArgument("require_session", default_value="false"),
        DeclareLaunchArgument("ready_min_fusion_updates", default_value="5"),
        DeclareLaunchArgument("use_camera_info", default_value="true"),
        DeclareLaunchArgument("position_cmd_topic", default_value="/boom_birds/ego/position_cmd"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("map_size_x", default_value="40.0"),
        DeclareLaunchArgument("map_size_y", default_value="40.0"),
        DeclareLaunchArgument("map_size_z", default_value="6.0"),
        DeclareLaunchArgument("cx", default_value="-1.0"),
        DeclareLaunchArgument("cy", default_value="-1.0"),
        DeclareLaunchArgument("fx", default_value="-1.0"),
        DeclareLaunchArgument("fy", default_value="-1.0"),
        DeclareLaunchArgument("compute_budget_s", default_value="0.0"),
        DeclareLaunchArgument("start_velocity_tolerance", default_value="0.0"),
        DeclareLaunchArgument("max_vel", default_value="1.0"),
        DeclareLaunchArgument("max_acc", default_value="1.0"),
        DeclareLaunchArgument("goal_x", default_value="2.5"),
        DeclareLaunchArgument("goal_y", default_value="1.2"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),
        DeclareLaunchArgument("obstacles_inflation", default_value="0.15"),
        DeclareLaunchArgument("obstacle_clearance", default_value="0.5"),
        DeclareLaunchArgument("depth_pose_tolerance_s", default_value="0.03"),
        DeclareLaunchArgument("resolution", default_value="0.1"),
        DeclareLaunchArgument("max_ray_length", default_value="4.5"),
    ]

    planner = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name=["drone_", drone_id, "_ego_planner_node"],
        output="screen",
        remappings=[
            ("/boom_birds/planner/request", LaunchConfiguration("planner_request_topic")),
            ("/boom_birds/planner/executor_status", LaunchConfiguration("executor_status_topic")),
            ("/boom_birds/planner/status", LaunchConfiguration("planner_status_topic")),
            ("/boom_birds/planner/command", LaunchConfiguration("planner_command_topic")),
            ("/boom_birds/control/execution_status", LaunchConfiguration("control_execution_topic")),
            ("odom_world", LaunchConfiguration("odom_topic")),
            ("grid_map/depth", LaunchConfiguration("depth_topic")),
            ("grid_map/camera_info", LaunchConfiguration("camera_info_topic")),
            ("grid_map/pose", LaunchConfiguration("camera_pose_topic")),
            # 不启用点云地图路径：话题保留且无发布者，避免误接第二条地图输入
            ("grid_map/cloud", "/boom_birds/depth/xyz_unused"),
            ("grid_map/odom", LaunchConfiguration("odom_topic")),
            ("grid_map/occupancy", "/boom_birds/ego/grid_map/occupancy"),
            ("grid_map/occupancy_inflate", "/boom_birds/ego/grid_map/occupancy_inflate"),
            ("planning/bspline", "/boom_birds/ego/planning/bspline"),
            ("planning/data_display", "/boom_birds/ego/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/boom_birds/ego/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/boom_birds/ego/broadcast_bspline"),
        ],
        parameters=[
            {"fsm/flight_type": 2},
            {"project/local_target_search_radius_m": ParameterValue(LaunchConfiguration("local_target_search_radius_m"), value_type=float), "project/local_target_search_step_m": ParameterValue(LaunchConfiguration("local_target_search_step_m"), value_type=float)},
            {"project/require_session": ParameterValue(LaunchConfiguration("require_session"), value_type=bool), "project/world_frame": LaunchConfiguration("world_frame"), "project/authorization_timeout_s": ParameterValue(LaunchConfiguration("authorization_timeout_s"), value_type=float), "project/future_tolerance_s": ParameterValue(LaunchConfiguration("command_future_tolerance_s"), value_type=float)},
            {"grid_map/use_camera_info": ParameterValue(LaunchConfiguration("use_camera_info"), value_type=bool)},
            {"fsm/thresh_replan_time": 1.0},
            {"fsm/thresh_no_replan_meter": 1.0},
            {"fsm/planning_horizon": ParameterValue(LaunchConfiguration("planning_horizon"), value_type=float)},
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
            {"grid_map/depth_pose_tolerance_s": ParameterValue(LaunchConfiguration("depth_pose_tolerance_s"), value_type=float)},
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
            {"grid_map/visualization_truncate_height": 3.6},
            {"grid_map/show_occ_time": False},
            {"grid_map/pose_type": 1},
            {"grid_map/frame_id": LaunchConfiguration("world_frame")},
            {"grid_map/odom_depth_timeout": 1.0},
            {"manager/compute_budget_s": ParameterValue(LaunchConfiguration("compute_budget_s"), value_type=float)},
            {"manager/start_velocity_tolerance": ParameterValue(LaunchConfiguration("start_velocity_tolerance"), value_type=float)},
            {"manager/max_vel": max_vel},
            {"manager/max_acc": max_acc},
            {"manager/max_jerk": 3.0},
            {"manager/control_points_distance": 0.4},
            {"manager/feasibility_tolerance": 0.05},
            {"manager/planning_horizon": ParameterValue(LaunchConfiguration("planning_horizon"), value_type=float)},
            {"manager/use_distinctive_trajs": True},
            {"manager/drone_id": 0},
            {"optimization/full_path_collision_check": True},
            {"optimization/lambda_smooth": 1.0},
            {"optimization/lambda_collision": 0.5},
            {"optimization/lambda_feasibility": 0.1},
            {"optimization/lambda_fitness": 1.0},
            {"optimization/dist0": ParameterValue(LaunchConfiguration("obstacle_clearance"), value_type=float)},
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
            ("/boom_birds/planner/request", LaunchConfiguration("planner_request_topic")),
            ("/boom_birds/planner/executor_status", LaunchConfiguration("executor_status_topic")),
            ("/boom_birds/planner/status", LaunchConfiguration("planner_status_topic")),
            ("/boom_birds/planner/command", LaunchConfiguration("planner_command_topic")),
            ("/boom_birds/control/execution_status", LaunchConfiguration("control_execution_topic")),
            ("planning/bspline", "/boom_birds/ego/planning/bspline"),
            ("/position_cmd", LaunchConfiguration("position_cmd_topic")),
        ],
        parameters=[{"traj_server/time_forward": 1.0, "project/require_session": ParameterValue(LaunchConfiguration("require_session"), value_type=bool), "project/world_frame": LaunchConfiguration("world_frame"), "project/authorization_timeout_s": ParameterValue(LaunchConfiguration("authorization_timeout_s"), value_type=float), "project/future_tolerance_s": ParameterValue(LaunchConfiguration("command_future_tolerance_s"), value_type=float)}],
    )

    def select_geometry(context):
        dynamic = LaunchConfiguration("use_camera_info").perform(context).lower() == "true"
        values = {k: float(LaunchConfiguration(k).perform(context)) for k in ("fx", "fy", "cx", "cy")}
        if dynamic and any(v != -1.0 for v in values.values()):
            raise ValueError("CameraInfo and static intrinsics are mutually exclusive")
        if not dynamic:
            import math
            if not all(math.isfinite(v) for v in values.values()) or values["fx"] <= 0 or values["fy"] <= 0 or values["cx"] < 0 or values["cy"] < 0:
                raise ValueError("Static mode requires explicit fx, fy, cx, cy")
        return [planner, traj_server]

    return LaunchDescription(args + [OpaqueFunction(function=select_geometry)])
