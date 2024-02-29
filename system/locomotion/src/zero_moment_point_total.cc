#include "huron/locomotion/zero_moment_point_total.h"

namespace huron {

template <typename T>
ZeroMomentPointTotal<T>::ZeroMomentPointTotal(
  std::weak_ptr<const multibody::Frame<T>> zmp_frame,
  const std::vector<std::shared_ptr<ZeroMomentPoint<T>>>& zmp_vector)
  : ZeroMomentPoint<T>(std::move(zmp_frame), 0.0),
    zmp_vector_(zmp_vector) {}

template <typename T>
huron::Vector2<T> ZeroMomentPointTotal<T>::Eval(T& fz) {
  huron::Vector2<T> zmp;
  T num_x(0.0), num_y(0.0), den(0.0);
  for (auto& zmp_obj : zmp_vector_) {
    T fz_i;
    huron::Vector2<T> zmp_i = zmp_obj->Eval(fz_i);
    num_x += zmp_i.x() * fz_i;
    num_y += zmp_i.y() * fz_i;
    den += fz_i;
  }
  fz = den;
  if (den == T(0.0)) {
    zmp.setZero();
  } else {
    zmp.x() = num_x / den;
    zmp.y() = num_y / den;
  }
  return zmp;
}

#if HURON_USE_CASADI==1
template <> huron::Vector2<casadi::SX>
ZeroMomentPointTotal<casadi::SX>::Eval(casadi::SX& fz) {
  casadi::SX num_x(0.0), num_y(0.0), den(0.0);
  for (auto& zmp_obj : zmp_vector_) {
    casadi::SX fz_i;
    huron::Vector2<casadi::SX> zmp_i = zmp_obj->Eval(fz_i);
    num_x += zmp_i.x() * fz_i;
    num_y += zmp_i.y() * fz_i;
    den += fz_i;
  }
  fz = den;
  auto f1 = casadi::Function(
    "f1",
    {den},
    {0.0, 0.0});
  auto f2 = casadi::Function(
    "f2",
    {den},
    ([&]() {
      return std::vector<casadi::SX>{
        num_x / den,
        num_y / den};
    })());
  std::vector<casadi::SX> tmp_res = casadi::Function::if_else(
    "zmp_fsr_array_eval", f1, f2)(
      std::vector<casadi::SX>{
        casadi::SX::abs(den) - this->normal_force_threshold_,
        den});
  return huron::Vector2<casadi::SX>(tmp_res[0], tmp_res[1]);
}
#endif  // HURON_USE_CASADI

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointTotal)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPointTotal)
