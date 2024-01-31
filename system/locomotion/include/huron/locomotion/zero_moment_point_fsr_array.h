#pragma once

#include <memory>

#include "huron/locomotion/zero_moment_point.h"

namespace huron {

template <typename T>
class ZeroMomentPointFSRArray : public ZeroMomentPoint<T> {
 public:
  ZeroMomentPointFSRArray(
    std::weak_ptr<const multibody::Frame<T>> zmp_frame,
    double normal_force_threshold,
    std::shared_ptr<ForceSensingResistorArray<T>> fsr_array);
  ZeroMomentPointFSRArray(const ZeroMomentPointFSRArray&) = delete;
  ZeroMomentPointFSRArray& operator=(const ZeroMomentPointFSRArray&) = delete;
  ~ZeroMomentPointFSRArray() override = default;

  Eigen::Vector2d Eval(double& fz) override;

 private:
  std::shared_ptr<ForceSensingResistorArray<T>> fsr_array_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceSensingResistorArray)
