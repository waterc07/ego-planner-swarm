#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <functional>
#include <limits>
namespace ego_planner {
// Cubic start state fixes the first three controls; retain the remaining warm-start shape.
inline bool anchorCubicStart(Eigen::MatrixXd& controls, double interval,
                            const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                            const Eigen::Vector3d& acceleration) {
  if (controls.rows() != 3 || controls.cols() < 4 || !controls.allFinite() ||
      !position.allFinite() || !velocity.allFinite() || !acceleration.allFinite() ||
      !std::isfinite(interval) || interval <= 0) return false;
  const Eigen::Vector3d middle = position - acceleration * interval * interval / 6.0;
  controls.col(0) = middle - velocity * interval + acceleration * interval * interval / 2.0;
  controls.col(1) = middle;
  controls.col(2) = middle + velocity * interval + acceleration * interval * interval / 2.0;
  return controls.allFinite();
}
// Periodic refresh must not repeatedly discard the initial acceleration segment.
// Collision-triggered replanning is independent of this scheduling predicate.
inline bool periodicReplanDue(double elapsed, double interval, double duration,
                              double measured_progress, double control_spacing) {
  if (!std::isfinite(elapsed) || !std::isfinite(interval) || !std::isfinite(duration) ||
      !std::isfinite(measured_progress) || !std::isfinite(control_spacing) ||
      interval <= 0 || duration <= 0 || measured_progress < 0 || control_spacing <= 0)
    return true;
  return elapsed > interval &&
      (measured_progress >= control_spacing || elapsed >= duration - interval);
}
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
// Preserve geometry and reject dilation when its initial velocity would jump.
inline bool dilateWithStartVelocity(UniformBspline& curve, double ratio,
                                   const Eigen::Vector3d& velocity, double tolerance) {
  if (!velocity.allFinite() || !std::isfinite(tolerance) || tolerance < 0) return false;
  auto candidate = curve;
  if (!dilateCubicTime(candidate, ratio) ||
      (candidate.getDerivative().evaluateDeBoorT(0.0) - velocity).norm() > tolerance) return false;
  curve = candidate;
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
