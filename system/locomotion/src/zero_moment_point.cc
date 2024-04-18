#include "huron/locomotion/zero_moment_point.h"
#include "huron/sensors/force_sensing_resistor_array.h"
#include "huron/types.h"

namespace huron {

template <typename T>
ZeroMomentPoint<T>::ZeroMomentPoint(
  std::weak_ptr<const multibody::Frame<T>> zmp_frame,
  T normal_force_threshold)
  : zmp_frame_(std::move(zmp_frame)),
    normal_force_threshold_(normal_force_threshold) {
}

template <typename T>
huron::SE3<T> ZeroMomentPoint<T>::ZmpToWorld(const huron::Vector2<T>& zmp) const {
  huron::SE3<T> ret(Matrix3<T>::Identity(),
                    huron::Vector3<T>(zmp.x(), zmp.y(), 0.0));
  return ret;
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPoint)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPoint)
