// Boom_Birds 离线地图行为测试（task-2 D）
//
// 目的：对 plan_env/src/grid_map.cpp 的深度投影/占据更新行为做可复现的行为断言，覆盖：
//   S1 无效不清图   S2 无虚假占据   S3 像素索引正确   S4 里程计不覆盖   S5 超量程/匹配失败
// 特点：
//   * 不需要真实设备：只用 ROS 2 话题（grid_map/depth, grid_map/pose, grid_map/odom）驱动真实回调。
//   * 同一份测试源码同时用于"补丁前（基线 SHA 原始源码，-DBB_PREPATCH）"与"补丁后"两个二进制，
//     便于给出"补丁前失败 / 补丁后通过"的对照证据。
//   * 通过 #define private public 读取 GridMap 内部量（仅测试用）；为此先展开所有被 grid_map.h
//     间接包含的头文件，再定义宏，最后 #undef。
//
// 构建：见 plan_env/CMakeLists.txt 中 -DBB_BUILD_GRIDMAP_TESTS=ON 的两个目标。

#include <bits/stdc++.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#define private public
#include <plan_env/grid_map.h>
#undef private

namespace
{

int g_pass = 0;
int g_fail = 0;
std::string g_scene = "-";

void check(bool ok, const std::string &name, const std::string &detail)
{
  if (ok)
  {
    ++g_pass;
    printf("  [PASS] %s :: %s\n", name.c_str(), detail.c_str());
  }
  else
  {
    ++g_fail;
    printf("  [FAIL] %s :: %s\n", name.c_str(), detail.c_str());
  }
  fflush(stdout);
}

std::string f2s(double v)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "%.4f", v);
  return std::string(buf);
}

std::string v3s(const Eigen::Vector3d &v)
{
  return "(" + f2s(v(0)) + ", " + f2s(v(1)) + ", " + f2s(v(2)) + ")";
}

// 光学坐标系 -> 世界：光轴(z_cam) 指向世界 +X，x_cam 指向世界 -Y，y_cam 指向世界 -Z
// 即 R 的三列为 (x_cam, y_cam, z_cam) 在世界系下的表示，满足右手系。
Eigen::Matrix3d camRotation()
{
  Eigen::Matrix3d R;
  R << 0.0, 0.0, 1.0,
      -1.0, 0.0, 0.0,
       0.0, -1.0, 0.0;
  return R;
}

struct Scene
{
  Eigen::Vector3d C = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Matrix3d R = camRotation();
  double fx = 387.229248046875;
  double fy = 387.229248046875;
  double cx = 321.04638671875;
  double cy = 243.44969177246094;

  Eigen::Vector3d project(double u, double v, double z) const
  {
    Eigen::Vector3d p((u - cx) * z / fx, (v - cy) * z / fy, z);
    return R * p + C;
  }
};

struct Harness
{
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<GridMap> gm;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  Scene scene;
};

void spinMs(const rclcpp::Node::SharedPtr &node, int ms)
{
  const auto t0 = std::chrono::steady_clock::now();
  while (true)
  {
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    if (dt >= ms)
      break;
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

Harness makeHarness(const std::string &name, bool use_filter, int skip_pixel,
                    double max_ray = 4.5, double max_dist = 5.0)
{
  Harness h;
  std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("grid_map/resolution", 0.1),
      rclcpp::Parameter("grid_map/map_size_x", 20.0),
      rclcpp::Parameter("grid_map/map_size_y", 20.0),
      rclcpp::Parameter("grid_map/map_size_z", 5.0),
      rclcpp::Parameter("grid_map/local_update_range_x", 8.0),
      rclcpp::Parameter("grid_map/local_update_range_y", 8.0),
      rclcpp::Parameter("grid_map/local_update_range_z", 5.0),
      rclcpp::Parameter("grid_map/obstacles_inflation", 0.099),
      rclcpp::Parameter("grid_map/local_map_margin", 10),
      rclcpp::Parameter("grid_map/ground_height", -0.01),
      rclcpp::Parameter("grid_map/virtual_ceil_height", 4.9),
      rclcpp::Parameter("grid_map/visualization_truncate_height", 4.9),
      rclcpp::Parameter("grid_map/fx", h.scene.fx),
      rclcpp::Parameter("grid_map/fy", h.scene.fy),
      rclcpp::Parameter("grid_map/cx", h.scene.cx),
      rclcpp::Parameter("grid_map/cy", h.scene.cy),
      rclcpp::Parameter("grid_map/use_depth_filter", use_filter),
      rclcpp::Parameter("grid_map/depth_filter_tolerance", 0.15),
      rclcpp::Parameter("grid_map/depth_filter_maxdist", max_dist),
      rclcpp::Parameter("grid_map/depth_filter_mindist", 0.2),
      rclcpp::Parameter("grid_map/depth_filter_margin", 2),
      rclcpp::Parameter("grid_map/k_depth_scaling_factor", 1000.0),
      rclcpp::Parameter("grid_map/skip_pixel", static_cast<int64_t>(skip_pixel)),
      rclcpp::Parameter("grid_map/p_hit", 0.65),
      rclcpp::Parameter("grid_map/p_miss", 0.35),
      rclcpp::Parameter("grid_map/p_min", 0.12),
      rclcpp::Parameter("grid_map/p_max", 0.90),
      rclcpp::Parameter("grid_map/p_occ", 0.80),
      rclcpp::Parameter("grid_map/min_ray_length", 0.1),
      rclcpp::Parameter("grid_map/max_ray_length", max_ray),
      rclcpp::Parameter("grid_map/show_occ_time", false),
      rclcpp::Parameter("grid_map/pose_type", static_cast<int64_t>(1)),
      rclcpp::Parameter("grid_map/frame_id", std::string("world")),
      rclcpp::Parameter("grid_map/odom_depth_timeout", 10.0),
      rclcpp::Parameter("grid_map/virtual_ceil_yp", 4.9),
      rclcpp::Parameter("grid_map/virtual_ceil_yn", -0.01),
  };
  rclcpp::NodeOptions opt;
  opt.parameter_overrides(params);
  h.node = std::make_shared<rclcpp::Node>(name, opt);
  h.gm = std::make_shared<GridMap>();
  h.gm->initMap(h.node);
  h.depth_pub = h.node->create_publisher<sensor_msgs::msg::Image>("grid_map/depth", rclcpp::QoS(50).reliable());
  h.pose_pub = h.node->create_publisher<geometry_msgs::msg::PoseStamped>("grid_map/pose", rclcpp::QoS(25).reliable());
  h.odom_pub = h.node->create_publisher<nav_msgs::msg::Odometry>("grid_map/odom", rclcpp::QoS(10).reliable());
  spinMs(h.node, 30);
  return h;
}

// 以 32FC1（米制）发布一帧深度 + 同时间戳的相机位姿，并给回调/定时器留出处理时间
void publishFrame(Harness &h, const cv::Mat &depth32f, int spin_ms = 110)
{
  cv_bridge::CvImage img;
  img.header.stamp = h.node->now();
  img.header.frame_id = "world";
  img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  img.image = depth32f;
  auto msg = img.toImageMsg();
  h.depth_pub->publish(*msg);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose.position.x = h.scene.C(0);
  pose.pose.position.y = h.scene.C(1);
  pose.pose.position.z = h.scene.C(2);
  Eigen::Quaterniond q(h.scene.R);
  q.normalize();
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  h.pose_pub->publish(pose);

  spinMs(h.node, spin_ms);
}

void publishOdom(Harness &h, const Eigen::Vector3d &p, int spin_ms = 40)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = h.node->now();
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = p(0);
  odom.pose.pose.position.y = p(1);
  odom.pose.pose.position.z = p(2);
  odom.pose.pose.orientation.w = 1.0;
  h.odom_pub->publish(odom);
  spinMs(h.node, spin_ms);
}

cv::Mat constDepth(float z) { return cv::Mat(480, 640, CV_32FC1, cv::Scalar(z)); }

// 左右阶梯深度：u < 320 为 near_z，u >= 320 为 far_z（用于暴露"像素列号与深度错位"）
cv::Mat stepDepth(float near_z, float far_z)
{
  cv::Mat img(480, 640, CV_32FC1, cv::Scalar(near_z));
  for (int v = 0; v < img.rows; ++v)
  {
    float *row = img.ptr<float>(v);
    for (int u = 320; u < img.cols; ++u)
      row[u] = far_z;
  }
  return img;
}

// 全无效帧：NaN / 0 / 负值 各占 1/3（三者都会被 convertTo(CV_16UC1) 变成 0）
cv::Mat invalidDepth()
{
  cv::Mat img(480, 640, CV_32FC1, cv::Scalar(0.0f));
  for (int v = 0; v < img.rows; ++v)
  {
    float *row = img.ptr<float>(v);
    for (int u = 0; u < img.cols; ++u)
    {
      if (u % 3 == 0)
        row[u] = std::numeric_limits<float>::quiet_NaN();
      else if (u % 3 == 1)
        row[u] = 0.0f;
      else
        row[u] = -1.0f;
    }
  }
  return img;
}

int countOccupied(const std::shared_ptr<GridMap> &gm)
{
  int n = 0;
  for (size_t i = 0; i < gm->md_.occupancy_buffer_.size(); ++i)
    if (gm->md_.occupancy_buffer_[i] >= gm->mp_.min_occupancy_log_)
      ++n;
  return n;
}

bool anyOccupiedInBox(const std::shared_ptr<GridMap> &gm, const Eigen::Vector3d &c, double r)
{
  Eigen::Vector3i lo, hi;
  gm->posToIndex(c - Eigen::Vector3d(r, r, r), lo);
  gm->posToIndex(c + Eigen::Vector3d(r, r, r), hi);
  gm->boundIndex(lo);
  gm->boundIndex(hi);
  for (int x = lo(0); x <= hi(0); ++x)
    for (int y = lo(1); y <= hi(1); ++y)
      for (int z = lo(2); z <= hi(2); ++z)
      {
        int xx = x, yy = y, zz = z;
        if (gm->md_.occupancy_buffer_[gm->toAddress(xx, yy, zz)] >= gm->mp_.min_occupancy_log_)
          return true;
      }
  return false;
}

// 与最近一次投影点集合的最小距离（用于直接断言像素-深度对应关系）
double minDistToProjected(const std::shared_ptr<GridMap> &gm, const Eigen::Vector3d &p)
{
  double best = 1e18;
  for (int i = 0; i < gm->md_.proj_points_cnt; ++i)
    best = std::min(best, (gm->md_.proj_points_[i] - p).norm());
  return best;
}

void printVariant()
{
#ifdef BB_PREPATCH
  printf("VARIANT: PREPATCH (grid_map.cpp = 基线 a3e14dd 原始源码 + Jazzy cv_bridge 头文件名兼容 shim)\n");
#else
  printf("VARIANT: PATCHED (工作区当前源码)\n");
#endif
}

// ---------------------------------------------------------------- S1
// 场景1：无效深度不得清图（use_depth_filter=true 时原实现把 0 深度当作超量程自由射线）
void scenario1_filtered()
{
  g_scene = "S1[无效不清图,use_depth_filter=true,skip=2]";
  printf("\n=== %s ===\n", g_scene.c_str());
  Harness h = makeHarness("bb_s1_filter", true, 2);
  Scene &S = h.scene;
  const Eigen::Vector3d P3 = S.project(500, 240, 3.0);

  for (int i = 0; i < 14; ++i)
    publishFrame(h, constDepth(3.0f));
  const int n0 = countOccupied(h.gm);
  check(n0 > 0, "S1/建立障碍", "occupied voxels=" + std::to_string(n0));
  check(anyOccupiedInBox(h.gm, P3, 0.15), "S1/障碍存在", "P(u=500,v=240,z=3.0)=" + v3s(P3));

  for (int i = 0; i < 4; ++i)
    publishFrame(h, invalidDepth());
  const int n1 = countOccupied(h.gm);
  check(h.gm->md_.proj_points_cnt == 0, "S1/无效帧无投影点",
        "proj_points_cnt=" + std::to_string(h.gm->md_.proj_points_cnt));
  check(n1 >= n0, "S1/占据计数不下降", "before=" + std::to_string(n0) + " after=" + std::to_string(n1));
  check(anyOccupiedInBox(h.gm, P3, 0.15), "S1/障碍未被清除", "P=" + v3s(P3));
  check(!anyOccupiedInBox(h.gm, S.C, 0.25), "S1/相机原点无虚假占据", "C=" + v3s(S.C));
}

// ---------------------------------------------------------------- S1b
// 场景1b：use_depth_filter=false（本项目离线配置）时，无效帧不得改变占据集合
void scenario1_unfiltered()
{
  g_scene = "S1b[无效不清图,use_depth_filter=false,skip=2]";
  printf("\n=== %s ===\n", g_scene.c_str());
  Harness h = makeHarness("bb_s1b_nofilter", false, 2);
  Scene &S = h.scene;
  const Eigen::Vector3d P3 = S.project(500, 240, 3.0);

  for (int i = 0; i < 14; ++i)
    publishFrame(h, constDepth(3.0f));
  const int n0 = countOccupied(h.gm);
  check(n0 > 0, "S1b/建立障碍", "occupied voxels=" + std::to_string(n0));

  for (int i = 0; i < 4; ++i)
    publishFrame(h, invalidDepth());
  const int n1 = countOccupied(h.gm);
  check(h.gm->md_.proj_points_cnt == 0, "S1b/无效帧无投影点",
        "proj_points_cnt=" + std::to_string(h.gm->md_.proj_points_cnt));
  check(n1 == n0, "S1b/占据集合不被无效观测改变",
        "before=" + std::to_string(n0) + " after=" + std::to_string(n1));
  check(anyOccupiedInBox(h.gm, P3, 0.15), "S1b/障碍未被清除", "P=" + v3s(P3));
}

// ---------------------------------------------------------------- S2
// 场景2：全 0/全无效帧不得在相机原点附近产生占据
void scenario2_no_false_occupancy()
{
  g_scene = "S2[无虚假占据,use_depth_filter=false,skip=2]";
  printf("\n=== %s ===\n", g_scene.c_str());
  Harness h = makeHarness("bb_s2_nofalse", false, 2);
  Scene &S = h.scene;

  for (int i = 0; i < 10; ++i)
    publishFrame(h, invalidDepth());
  const int n = countOccupied(h.gm);
  check(h.gm->md_.proj_points_cnt == 0, "S2/无效帧无投影点",
        "proj_points_cnt=" + std::to_string(h.gm->md_.proj_points_cnt));
  check(n == 0, "S2/全图无占据体素", "occupied voxels=" + std::to_string(n));
  check(h.gm->getOccupancy(S.C) == 0, "S2/相机所在体素未占据", "getOccupancy(C)=" + std::to_string(h.gm->getOccupancy(S.C)));
  check(!anyOccupiedInBox(h.gm, S.C, 0.5), "S2/相机 0.5m 内无占据", "C=" + v3s(S.C));
}

// ---------------------------------------------------------------- S3
// 场景3：u 与深度必须按列对应，skip_pixel=1 与 2 两组
void scenario3_index(int skip_pixel)
{
  g_scene = "S3[索引正确,skip_pixel=" + std::to_string(skip_pixel) + "]";
  printf("\n=== %s ===\n", g_scene.c_str());
  Harness h = makeHarness("bb_s3_skip" + std::to_string(skip_pixel), false, skip_pixel);
  Scene &S = h.scene;

  const float near_z = 1.5f, far_z = 3.0f;
  cv::Mat img = stepDepth(near_z, far_z);

  publishFrame(h, img, 130);
  // 直接核对投影点：像素 (500,240) 的解析落点必须与某个投影点重合
  const Eigen::Vector3d P_far = S.project(500, 240, 3.0);
  const Eigen::Vector3d P_near = S.project(100, 240, 1.5);
  const double d_far = minDistToProjected(h.gm, P_far);
  const double d_near = minDistToProjected(h.gm, P_near);
  check(d_far < 1e-9, "S3/投影点匹配像素(500,240)@3.0m",
        "min|proj-P|=" + f2s(d_far) + " P=" + v3s(P_far));
  check(d_near < 1e-9, "S3/投影点匹配像素(100,240)@1.5m",
        "min|proj-P|=" + f2s(d_near) + " P=" + v3s(P_near));

  for (int i = 0; i < 13; ++i)
    publishFrame(h, img);

  // 缺陷索引下像素(500,240)会用到第 250 列的深度(near_z) -> 落点 P_bug
  const Eigen::Vector3d P_bug = S.project(500, 240, 1.5);
  check(anyOccupiedInBox(h.gm, P_far, 0.15), "S3/远端真实落点被占据", "P=" + v3s(P_far));
  check(!anyOccupiedInBox(h.gm, P_bug, 0.15), "S3/错位落点无占据", "P_bug=" + v3s(P_bug));
  check(anyOccupiedInBox(h.gm, P_near, 0.15), "S3/近端落点被占据", "P=" + v3s(P_near));
}

// ---------------------------------------------------------------- S4
// 场景4：已有有效深度观测后，独立 odom 回调不得覆盖 camera_pos_
void scenario4_odom_not_overwrite()
{
  g_scene = "S4[里程计不覆盖]";
  printf("\n=== %s ===\n", g_scene.c_str());

  // 4a: 尚无有效深度观测时，odom 仍应能预置 camera_pos_（保证原有回退行为不被破坏）
  {
    Harness h = makeHarness("bb_s4a_preset", false, 2);
    const Eigen::Vector3d C2(6.0, 6.0, 3.0);
    publishOdom(h, C2, 80);
    const double d = (h.gm->md_.camera_pos_ - C2).norm();
    check(d < 1e-9, "S4a/无深度观测时 odom 预置 camera_pos_",
          "|camera_pos-C_odom|=" + f2s(d) + " camera_pos=" + v3s(h.gm->md_.camera_pos_));
  }

  // 4b: 已有有效深度观测后，odom 不得覆盖
  Harness h = makeHarness("bb_s4b_nowover", false, 2);
  Scene &S = h.scene;
  for (int i = 0; i < 6; ++i)
    publishFrame(h, constDepth(3.0f));
  const Eigen::Vector3d c_before = h.gm->md_.camera_pos_;
  const int n_before = countOccupied(h.gm);
  check((c_before - S.C).norm() < 1e-9, "S4b/位姿来自 depthPoseCallback",
        "camera_pos=" + v3s(c_before));

  const Eigen::Vector3d C_odom(6.0, 6.0, 3.0);
  for (int i = 0; i < 5; ++i)
    publishOdom(h, C_odom);
  spinMs(h.node, 150);

  const double d = (h.gm->md_.camera_pos_ - c_before).norm();
  check(d < 1e-9, "S4b/odom 未覆盖 camera_pos_",
        "|camera_pos-before|=" + f2s(d) + " camera_pos=" + v3s(h.gm->md_.camera_pos_));
  check(h.gm->md_.has_odom_, "S4b/odom 回调确实执行", "has_odom_=true");
  check(countOccupied(h.gm) == n_before, "S4b/odom 不改变占据",
        "before=" + std::to_string(n_before) + " after=" + std::to_string(countOccupied(h.gm)));
#ifdef BB_PREPATCH
  printf("  [info] PREPATCH: |camera_pos-C_odom|=%s\n", f2s((h.gm->md_.camera_pos_ - C_odom).norm()).c_str());
#else
  printf("  [info] PATCHED: has_valid_depth_obs_=%d\n", (int)h.gm->md_.has_valid_depth_obs_);
#endif
}

// ---------------------------------------------------------------- S5
// 场景5：只有"经过验证的超量程观测"才允许延伸清空；匹配失败/无效观测不延伸
void scenario5_overrange()
{
  g_scene = "S5[超量程/匹配失败]";
  printf("\n=== %s ===\n", g_scene.c_str());

  const double max_ray = 4.5;
  Scene S;

  // 5a: 有效且 z>max_ray_length -> 允许沿射线清空（保留既有意图）
  {
    Harness h = makeHarness("bb_s5a_overrange", true, 2, max_ray, 5.0);
    for (int i = 0; i < 14; ++i)
      publishFrame(h, constDepth(3.0f));
    const int n0 = countOccupied(h.gm);
    const Eigen::Vector3d P3 = S.project(500, 240, 3.0);
    check(n0 > 0, "S5a/建立障碍", "occupied voxels=" + std::to_string(n0));

    for (int i = 0; i < 3; ++i)
      publishFrame(h, constDepth((float)(max_ray + 0.3)));
    const int n1 = countOccupied(h.gm);
    // 注意：raycastProcess 在同一帧内用 flag_traverse_ 对已访问体素提前 break（上游既有优化，未修改），
    // 因此"某个具体体素是否被清空"依赖像素遍历顺序，不能作为断言；此处只断言聚合效应：
    // 有效超量程观测确实把射线路径上的空间大幅清空。
    const double cleared = (n0 > 0) ? (1.0 - double(n1) / double(n0)) : 0.0;
    check(n1 < n0 && cleared >= 0.3, "S5a/有效超量程观测清空射线路径",
          "before=" + std::to_string(n0) + " after=" + std::to_string(n1) + " cleared=" + f2s(100.0 * cleared) + "%");
    printf("  [info] S5a: 3.0m 方向体素盒仍占据=%d（受 flag_traverse_ 提前 break 影响，仅记录不断言）\n",
           (int)anyOccupiedInBox(h.gm, P3, 0.15));
  }

  // 5b: 匹配失败（0 深度/无效）不得延伸清空 -> 障碍必须保留
  {
    Harness h = makeHarness("bb_s5b_nomatch", true, 2, max_ray, 5.0);
    for (int i = 0; i < 14; ++i)
      publishFrame(h, constDepth(3.0f));
    const int n0 = countOccupied(h.gm);
    const Eigen::Vector3d P3 = S.project(500, 240, 3.0);
    check(n0 > 0 && anyOccupiedInBox(h.gm, P3, 0.15), "S5b/建立障碍",
          "occupied=" + std::to_string(n0) + " P=" + v3s(P3));

    for (int i = 0; i < 3; ++i)
      publishFrame(h, invalidDepth());
    const int n1 = countOccupied(h.gm);
    check(n1 >= n0, "S5b/匹配失败不延伸清空",
          "before=" + std::to_string(n0) + " after=" + std::to_string(n1));
    check(anyOccupiedInBox(h.gm, P3, 0.15), "S5b/障碍保留", "P=" + v3s(P3));
  }

  // 5c: use_depth_filter=false 下的有效超量程同样应清空（射线逻辑未改）
  {
    Harness h = makeHarness("bb_s5c_overrange_nofilter", false, 2, max_ray, 5.0);
    for (int i = 0; i < 14; ++i)
      publishFrame(h, constDepth(3.0f));
    const int n0 = countOccupied(h.gm);
    for (int i = 0; i < 3; ++i)
      publishFrame(h, constDepth((float)(max_ray + 0.3)));
    const int n1 = countOccupied(h.gm);
    check(n0 > 0 && n1 < n0, "S5c/无过滤分支超量程清空",
          "before=" + std::to_string(n0) + " after=" + std::to_string(n1));
  }
}

#ifndef BB_PREPATCH
void scenario6_readiness()
{
  Harness h = makeHarness("bb_s6_readiness", false, 2);
  check(!h.gm->mapReady(1), "S6/no input stays gated", "");
  for (int i = 0; i < 5; ++i)
    publishFrame(h, constDepth(3.0f));
  check(h.gm->md_.depth_fusion_updates_ == 5, "S6/completed fusion count", "");
  check(countOccupied(h.gm) == 0 && !h.gm->mapReady(5),
        "S6/five hits cannot open an empty map", "");
  publishFrame(h, constDepth(3.0f));
  check(countOccupied(h.gm) > 0 && h.gm->mapReady(5),
        "S6/sixth hit establishes occupancy before ready", "");
  check(!h.gm->mapReady(20), "S6/configured larger warmup preserved", "");
  const auto before = h.gm->md_.depth_fusion_updates_;
  publishFrame(h, invalidDepth());
  check(h.gm->md_.depth_fusion_updates_ == before,
        "S6/invalid frame cannot advance readiness", "");
}
#endif

} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  printVariant();
  printf("BB_GRIDMAP_TEST START\n");

  scenario1_filtered();
  scenario1_unfiltered();
  scenario2_no_false_occupancy();
  scenario3_index(1);
  scenario3_index(2);
  scenario4_odom_not_overwrite();
  scenario5_overrange();
#ifndef BB_PREPATCH
  scenario6_readiness();
#endif

  printf("\nBB_GRIDMAP_TEST SUMMARY: pass=%d fail=%d\n", g_pass, g_fail);
  rclcpp::shutdown();
  return g_fail == 0 ? 0 : 1;
}
