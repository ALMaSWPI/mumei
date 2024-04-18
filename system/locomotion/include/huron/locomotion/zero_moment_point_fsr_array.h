#pragma once

#include <memory>

#include "huron/locomotion/zero_moment_point.h"
#include "huron/types.h"

namespace huron {

template <typename T>
class ZeroMomentPointFSRArray : public ZeroMomentPoint<T> {
 public:
  ZeroMomentPointFSRArray(
    std::weak_ptr<const multibody::Frame<T>> zmp_frame,
    T normal_force_threshold,
    std::shared_ptr<ForceSensingResistorArray<T>> fsr_array);
  ZeroMomentPointFSRArray(const ZeroMomentPointFSRArray&) = delete;
  ZeroMomentPointFSRArray& operator=(const ZeroMomentPointFSRArray&) = delete;
  ~ZeroMomentPointFSRArray() override = default;

  huron::Vector2<T> Eval(T& fz) override;

 private:
  std::shared_ptr<ForceSensingResistorArray<T>> fsr_array_;
};

}  // namespace huron

// Template specializations
#ifdef HURON_USE_CASADI

template <> huron::Vector2<casadi::SX>
huron::ZeroMomentPointFSRArray<casadi::SX>::Eval(casadi::SX& fz);

#endif  // HURON_USE_CASADI

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointFSRArray)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPointFSRArray)
