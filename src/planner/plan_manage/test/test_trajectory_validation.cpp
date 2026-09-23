#include "ego_planner/trajectory_validation.h"
#include <iostream>
#include <stdexcept>
using namespace ego_planner;
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int main() {
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
