#pragma once

#include <memory>
#include <vector>

#include "huron/locomotion/zero_moment_point.h"

namespace huron {

template <typename T>
class ZeroMomentPointFTSensor : public ZeroMomentPoint<T> {
 public:
  ZeroMomentPointFTSensor(
    std::weak_ptr<const multibody::Frame<T>> zmp_frame,
    T normal_force_threshold,
    const std::vector<std::shared_ptr<ForceTorqueSensor<T>>>& ft_sensors);
  ZeroMomentPointFTSensor(const ZeroMomentPointFTSensor&) = delete;
  ZeroMomentPointFTSensor& operator=(const ZeroMomentPointFTSensor&) = delete;
  ~ZeroMomentPointFTSensor() override = default;

  huron::Vector2<T> Eval(T& fz) override;

 private:
  std::vector<std::shared_ptr<ForceTorqueSensor<T>>> ft_sensors_;
};

}  // namespace huron

// Template specializations
#ifdef HURON_USE_CASADI

template <> huron::Vector2<casadi::SX>
huron::ZeroMomentPointFTSensor<casadi::SX>::Eval(casadi::SX& fz);

#endif  // HURON_USE_CASADI

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointFTSensor)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPointFTSensor)
