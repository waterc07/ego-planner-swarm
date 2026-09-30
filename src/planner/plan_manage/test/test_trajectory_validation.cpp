#include "ego_planner/trajectory_validation.h"
#include "ego_planner/local_target.h"
#include <iostream>
#include <stdexcept>
using namespace ego_planner;
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int main() {
  const Eigen::Vector3d start(0,0,1), desired(3,0,1), goal(10,0,1);
  const auto empty = [](const Eigen::Vector3d&) { return true; };
  auto target = selectLocalTarget(start, desired, goal, 1.5, 0.2, empty);
  require(target && (*target - desired).norm() == 0, "free nominal target changed");
  const auto occupied = [&desired](const Eigen::Vector3d& p) { return (p - desired).norm() > 0.8; };
  target = selectLocalTarget(start, desired, goal, 1.5, 0.2, occupied);
  require(target && occupied(*target) && (*target - desired).norm() <= 1.5 &&
          (goal - *target).norm() < (goal - start).norm(), "local target did not avoid occupied endpoint");
  require(!selectLocalTarget(start, desired, desired, 1.5, 0.2, occupied), "final goal silently moved");
  require(!selectLocalTarget(start, desired, goal, 1.5, 0.2,
                             [](const Eigen::Vector3d&) { return false; }), "fully occupied map accepted");
  require(!selectLocalTarget(start, desired, goal, 1.5, 0.0001, empty), "unbounded target search accepted");
  int repaired=0, checked=0;
  require(validateWithRepairs(3, [&]{++checked; return repaired==3;},
                             [&]{++repaired; return true;}), "last repair not checked");
  require(checked==4 && repaired==3, "incorrect repair budget");
  repaired=checked=0;
  require(!validateWithRepairs(3, [&]{++checked;return false;},
                              [&]{++repaired;return false;}), "failed repair accepted");
  require(checked==1 && repaired==1, "consumed failed optimization");
  require(!validateWithRepairs(3, []{return false;}, []{return true;}), "unbounded retries");

  Eigen::MatrixXd pts(3,6);
  for(int i=0;i<6;++i) pts.col(i)=Eigen::Vector3d(i,i,0.5);
  UniformBspline curve(pts,3,1.0);
  auto bounds=trajectoryBounds(curve);
  require(bounds.velocity>1.4, "diagonal speed exceeds per-axis limit");
  auto stretched = curve;
  require(dilateCubicTime(stretched, 2.0), "time dilation failed");
  const auto stretched_bounds = trajectoryBounds(stretched);
  require(std::abs(stretched_bounds.velocity * 2 - bounds.velocity) < 1e-10 &&
          std::abs(stretched_bounds.acceleration * 4 - bounds.acceleration) < 1e-10,
          "time dilation derivative scaling wrong");
  for (int i=0;i<=100;++i) {
    const double t=curve.getTimeSum()*i/100.;
    require((curve.evaluateDeBoorT(t)-stretched.evaluateDeBoorT(2*t)).norm()<1e-10,
            "time dilation changed geometric path");
  }
  require(!dilateCubicTime(stretched, 0.5), "unsafe speed-up accepted");
  auto moving = curve;
  const auto initial_velocity = curve.getDerivative().evaluateDeBoorT(0.0);
  require(!dilateWithStartVelocity(moving, 2.0, initial_velocity, 0.3),
          "dilation bypassed velocity continuity");
  require(std::abs(moving.getTimeSum() - curve.getTimeSum()) < 1e-10,
          "failed dilation mutated candidate");
  require(dilateWithStartVelocity(moving, 1.1, initial_velocity, 0.3),
          "bounded moving-start dilation rejected");
  require((moving.getDerivative().evaluateDeBoorT(0.0) - initial_velocity).norm() <= 0.3,
          "moving start exceeded handoff tolerance");

  auto vel=curve.getDerivative();
  for(int i=0;i<=1000;++i)
    require(vel.evaluateDeBoorT(curve.getTimeSum()*i/1000.).norm()<=bounds.velocity+1e-10,
            "control hull fails to bound derivative");
  require(sweptPathClear(curve,bounds.velocity,0.1,Eigen::Vector3d::Zero(),
                        [](const Eigen::Vector3d&){return false;}), "empty map rejected");
  require(!sweptPathClear(curve,bounds.velocity,0.1,Eigen::Vector3d::Zero(),
                         [](const Eigen::Vector3d& p){return p.x()>2.0 && p.x()<2.1;}),
          "thin occupied voxel crossed");
  require(!sweptPathClear(curve,bounds.velocity,0.1,Eigen::Vector3d::Zero(),
                         [](const Eigen::Vector3d& p){return p.x()>=3.5;}),
          "out-of-map tail accepted");
  pts(0,2)=std::numeric_limits<double>::quiet_NaN();
  UniformBspline bad(pts,3,1.0);
  require(!std::isfinite(trajectoryBounds(bad).velocity), "NaN bound accepted");
  require(!sweptPathClear(bad,1.0,0.1,Eigen::Vector3d::Zero(),
                         [](const Eigen::Vector3d&){return false;}), "NaN path accepted");
  std::cout << "PASS: repair failure/last retry, whole-interval bounds, swept collision, invalid data\n";
}
