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
    double normal_force_threshold,
    const std::vector<std::shared_ptr<ForceTorqueSensor<T>>>& ft_sensors);
  ZeroMomentPointFTSensor(const ZeroMomentPointFTSensor&) = delete;
  ZeroMomentPointFTSensor& operator=(const ZeroMomentPointFTSensor&) = delete;
  ~ZeroMomentPointFTSensor() override = default;

  Eigen::Vector2d Eval(double& fz) override;

 private:
  std::vector<std::shared_ptr<ForceTorqueSensor<T>>> ft_sensors_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointFTSensor)
