#include "huron/locomotion/zero_moment_point_total.h"

namespace huron {

template <typename T>
ZeroMomentPointTotal<T>::ZeroMomentPointTotal(
  std::weak_ptr<const multibody::Frame<T>> zmp_frame,
  const std::vector<std::shared_ptr<ZeroMomentPoint<T>>>& zmp_vector)
  : ZeroMomentPoint<T>(std::move(zmp_frame), 0.0),
    zmp_vector_(zmp_vector) {}

template <typename T>
Eigen::Vector2d ZeroMomentPointTotal<T>::Eval(double& fz) {
  Eigen::Vector2d zmp;
  double num_x = 0.0, num_y = 0.0, den = 0.0;
  for (auto& zmp_obj : zmp_vector_) {
    double fz_i;
    Eigen::Vector2d zmp_i = zmp_obj->Eval(fz_i);
    num_x += zmp_i.x() * fz_i;
    num_y += zmp_i.y() * fz_i;
    den += fz_i;
  }
  fz = den;
  if (den == 0.0) {
    zmp.setZero();
  } else {
    zmp.x() = num_x / den;
    zmp.y() = num_y / den;
  }
  return zmp;
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointTotal)
