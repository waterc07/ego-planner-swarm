#pragma once
#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>

namespace ego_planner {
// Adjust intermediate targets only. The optimizer still validates the entire curve.
inline std::optional<Eigen::Vector3d> selectLocalTarget(
    const Eigen::Vector3d& start, const Eigen::Vector3d& desired,
    const Eigen::Vector3d& goal, double radius, double step,
    const std::function<bool(const Eigen::Vector3d&)>& free) {
  if (!start.allFinite() || !desired.allFinite() || !goal.allFinite() ||
      !std::isfinite(radius) || !std::isfinite(step) || step <= 0 || radius < step || radius / step > 100)
    return std::nullopt;
  if (free(desired)) return desired;
  if ((desired - goal).norm() < 1e-6) return std::nullopt;
  Eigen::Vector3d forward = goal - start;
  forward.z() = 0;
  if (forward.norm() < 1e-6) forward = Eigen::Vector3d::UnitX();
  else forward.normalize();
  const Eigen::Vector3d side(-forward.y(), forward.x(), 0);
  const double remaining = (goal - start).norm();
  double best = std::numeric_limits<double>::infinity();
  std::optional<Eigen::Vector3d> selected;
  for (int ring = 1; ring * step <= radius + 1e-9; ++ring) {
    const double distance = ring * step;
    for (int angle = 0; angle < 32; ++angle) {
      const double theta = angle * 2 * std::acos(-1.) / 32;
      const Eigen::Vector3d candidate = desired + distance *
          (std::cos(theta) * forward + std::sin(theta) * side);
      if ((goal - candidate).norm() >= remaining - 0.1 ||
          (candidate - start).dot(goal - start) <= 0 || !free(candidate)) continue;
      const double score = distance * distance + 0.1 * (goal - candidate).squaredNorm();
      if (score < best) { best = score; selected = candidate; }
    }
  }
  return selected;
}
}
