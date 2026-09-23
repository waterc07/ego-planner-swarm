#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <functional>
#include <limits>
namespace ego_planner {
// Nonnegative basis sums to one: control-point norms bound the ENTIRE interval.
inline double derivativeNormBound(UniformBspline curve) {
  const auto points = curve.getControlPoint();
  if (points.cols() == 0 || !points.allFinite())
    return std::numeric_limits<double>::infinity();
  return points.colwise().norm().maxCoeff();
}
struct TrajectoryBounds { double velocity; double acceleration; };
inline TrajectoryBounds trajectoryBounds(UniformBspline curve) {
  auto velocity = curve.getDerivative();
  return {derivativeNormBound(velocity), derivativeNormBound(velocity.getDerivative())};
}
// Uniform dilation preserves the geometric path exactly. Caller must establish
// that rescaling endpoint derivatives does not violate nonzero boundary states.
inline bool dilateCubicTime(UniformBspline& curve, double ratio) {
  if (!std::isfinite(ratio) || ratio < 1.0) return false;
  auto knots = curve.getKnot();
  const auto points = curve.getControlPoint();
  if (points.cols() < 4 || knots.size() != points.cols() + 4 ||
      !points.allFinite() || !knots.allFinite()) return false;
  const double origin = knots(3);
  knots = ((knots.array() - origin) * ratio + origin).matrix();
  const double interval = curve.getInterval() * ratio;
  if (!knots.allFinite() || !std::isfinite(interval) || interval <= 0) return false;
  curve = UniformBspline(points, 3, interval);
  curve.setKnot(knots);
  return true;
}
// Entire interval lies within speed_bound*dt/2 of its midpoint.
// Query every voxel intersecting that enclosing cube in the inflated map.
inline bool sweptPathClear(UniformBspline curve, double speed_bound,
                           double resolution, const Eigen::Vector3d& origin,
                           const std::function<bool(const Eigen::Vector3d&)>& occupied) {
  const double duration = curve.getTimeSum();
  if (!std::isfinite(duration) || duration <= 0 || !std::isfinite(speed_bound) ||
      speed_bound < 0 || !std::isfinite(resolution) || resolution <= 0 ||
      !curve.getControlPoint().allFinite()) return false;
  const double count = std::max(1.0, std::ceil(2.0 * speed_bound * duration / resolution));
  if (count > 100000) return false;
  const int n = static_cast<int>(count);
  const double dt = duration / n;
  const double radius = speed_bound * dt * 0.5 + 1e-9;
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d p = curve.evaluateDeBoorT((i + 0.5) * dt);
    if (!p.allFinite()) return false;
    const Eigen::Array3i lo = ((p.array() - radius - origin.array()) / resolution).floor().cast<int>();
    const Eigen::Array3i hi = ((p.array() + radius - origin.array()) / resolution).floor().cast<int>();
    for (int x=lo.x(); x<=hi.x(); ++x)
      for (int y=lo.y(); y<=hi.y(); ++y)
        for (int z=lo.z(); z<=hi.z(); ++z) {
          const Eigen::Vector3d center = origin + resolution * Eigen::Vector3d(x+0.5,y+0.5,z+0.5);
          if (occupied(center)) return false;
        }
  }
  return true;
}
// Failed repair is never consumed; check the final successful repair too.
template<class Check, class Repair>
bool validateWithRepairs(int repairs, Check check, Repair repair) {
  for (int attempt=0; ; ++attempt) {
    if (check()) return true;
    if (attempt >= repairs || !repair()) return false;
  }
}
}
