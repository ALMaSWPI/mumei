#include "huron/locomotion/zero_moment_point_fsr_array.h"
#include "huron/math/abs.h"

namespace huron {

template <typename T>
ZeroMomentPointFSRArray<T>::ZeroMomentPointFSRArray(
  std::weak_ptr<const multibody::Frame<T>> zmp_frame,
  T normal_force_threshold,
  std::shared_ptr<ForceSensingResistorArray<T>> fsr_array)
  : ZeroMomentPoint<T>(std::move(zmp_frame), normal_force_threshold),
    fsr_array_(std::move(fsr_array)) {
}

template <typename T>
huron::Vector2<T> ZeroMomentPointFSRArray<T>::Eval(T& fz) {
  huron::Vector2<T> zmp;
  huron::VectorX<T> fz_array = fsr_array_->GetValue();
  T sum_fz = fz_array.colwise().sum().value();
  fz = sum_fz;
  if (huron::abs<T>(sum_fz) < this->normal_force_threshold_) {
    zmp.setZero();
  } else {
    T num_x(0.0), num_y(0.0);
    for (size_t i = 0; i < fsr_array_->num_sensors(); ++i) {
      num_x += fz_array(i) * fsr_array_->GetSensorPose(i).translation().x();
      num_y += fz_array(i) * fsr_array_->GetSensorPose(i).translation().y();
    }
    zmp(0) = num_x / sum_fz;
    zmp(1) = num_y / sum_fz;
  }
  return zmp;
}
#if HURON_USE_CASADI==1
template <> huron::Vector2<casadi::SX>
ZeroMomentPointFSRArray<casadi::SX>::Eval(casadi::SX& fz) {
  huron::VectorX<casadi::SX> fz_array = fsr_array_->GetValue();
  casadi::SX sum_fz = fz_array.colwise().sum().value();
  fz = sum_fz;
  auto f1 = casadi::Function(
    "f1",
    {sum_fz},
    {0.0, 0.0});
  auto f2 = casadi::Function(
    "f2",
    {sum_fz},
    ([&]() {
      std::vector<casadi::SX> zmp(0.0, 0.0);
      for (size_t i = 0; i < fsr_array_->num_sensors(); ++i) {
        zmp[0] += fz_array(i) * fsr_array_->GetSensorPose(i).translation().x();
        zmp[1] += fz_array(i) * fsr_array_->GetSensorPose(i).translation().y();
      }
      zmp[0] /= sum_fz;
      zmp[1] /= sum_fz;
      return zmp;
    })());
  std::vector<casadi::SX> tmp_res = casadi::Function::if_else(
    "zmp_fsr_array_eval", f1, f2)(
      std::vector<casadi::SX>{
        casadi::SX::abs(sum_fz) - this->normal_force_threshold_,
        sum_fz});
  return huron::Vector2<casadi::SX>(tmp_res[0], tmp_res[1]);
}
#endif  // HURON_USE_CASADI

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointFSRArray)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPointFSRArray)
