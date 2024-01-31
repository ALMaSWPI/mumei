#include "huron/locomotion/zero_moment_point.h"
#include "huron/sensors/force_sensing_resistor_array.h"
#include "huron/types.h"

namespace huron {

template <typename T>
ZeroMomentPoint<T>::ZeroMomentPoint(
  std::weak_ptr<const multibody::Frame<T>> zmp_frame,
  double normal_force_threshold)
  : zmp_frame_(std::move(zmp_frame)),
    normal_force_threshold_(normal_force_threshold) {
}

template <typename T>
Eigen::Affine3d ZeroMomentPoint<T>::ZmpToWorld(const Eigen::Vector2d& zmp) const {
  Eigen::Affine3d ret;
  ret.translate(Eigen::Vector3d(zmp.x(), zmp.y(), 0.0));
  return ret;
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPoint)
