#include "mumei/locomotion/zero_moment_point.h"
#include "mumei/sensors/force_sensing_resistor_array.h"
#include "mumei/types.h"
namespace mumei {

ZeroMomentPoint::ZeroMomentPoint(
  std::weak_ptr<const multibody::Frame> zmp_frame,
  double normal_force_threshold)
  : zmp_frame_(std::move(zmp_frame)),
    normal_force_threshold_(normal_force_threshold) {
}

Eigen::Affine3d ZeroMomentPoint::ZmpToWorld(const Eigen::Vector2d& zmp) const {
  Eigen::Affine3d ret;
  ret.translate(Eigen::Vector3d(zmp.x(), zmp.y(), 0.0));
  return ret;
}

}  // namespace mumei
