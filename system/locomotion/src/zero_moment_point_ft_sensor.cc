#include "huron/locomotion/zero_moment_point_ft_sensor.h"
#include "huron/math/abs.h"

namespace huron {

template <typename T>
ZeroMomentPointFTSensor<T>::ZeroMomentPointFTSensor(
  std::weak_ptr<const multibody::Frame<T>> zmp_frame,
  T normal_force_threshold,
  const std::vector<std::shared_ptr<ForceTorqueSensor<T>>>& ft_sensors)
  : ZeroMomentPoint<T>(std::move(zmp_frame), normal_force_threshold),
    ft_sensors_(ft_sensors) {
}

template <typename T>
huron::Vector2<T> ZeroMomentPointFTSensor<T>::Eval(T& fz) {
  huron::Vector2<T> zmp;
  T num_x(0.0), num_y(0.0), den(0.0);
  for (auto& ft_sensor : ft_sensors_) {
    huron::SE3<T> zmp_to_sensor = this->zmp_frame_.lock()->GetTransformToFrame(
      *ft_sensor->GetSensorFrame().lock());
    huron::SE3<T> zmp_frame_pose = this->zmp_frame_.lock()->GetTransformInWorld();
    huron::Vector6<T> w = ft_sensor->GetValue();
    w = zmp_to_sensor.Inverse().AdjointAction().transpose() * w;
    num_x +=
      (-w(4) -
      w(0)*(zmp_to_sensor.translation().z() - zmp_frame_pose.translation().z())
      + zmp_to_sensor.translation().x()*w(2));
    num_y +=
      (w(3) -
      w(1)*(zmp_to_sensor.translation().z() - zmp_frame_pose.translation().z())
      + zmp_to_sensor.translation().y()*w(2));
    den += w(2);
  }
  if (huron::abs<T>(den) < this->normal_force_threshold_) {
    zmp.setZero();
  } else {
    zmp.x() = num_x / den;
    zmp.y() = num_y / den;
  }
  fz = den;
  return zmp;
}

#if HURON_USE_CASADI==1
template <> huron::Vector2<casadi::SX>
ZeroMomentPointFTSensor<casadi::SX>::Eval(casadi::SX& fz) {
  huron::Vector2<casadi::SX> zmp;
  casadi::SX num_x(0.0), num_y(0.0), den(0.0);
  for (auto& ft_sensor : ft_sensors_) {
    huron::SE3<casadi::SX> zmp_to_sensor = this->zmp_frame_.lock()->GetTransformToFrame(
      *ft_sensor->GetSensorFrame().lock());
    huron::SE3<casadi::SX> zmp_frame_pose = this->zmp_frame_.lock()->GetTransformInWorld();
    huron::Vector6<casadi::SX> w = ft_sensor->GetValue();
    w = zmp_to_sensor.Inverse().AdjointAction().transpose() * w;
    num_x +=
      (-w(4) -
      w(0)*(zmp_to_sensor.translation().z() - zmp_frame_pose.translation().z())
      + zmp_to_sensor.translation().x()*w(2));
    num_y +=
      (w(3) -
      w(1)*(zmp_to_sensor.translation().z() - zmp_frame_pose.translation().z())
      + zmp_to_sensor.translation().y()*w(2));
    den += w(2);
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
#endif

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointFTSensor)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPointFTSensor)
