#pragma once

#include <memory>
#include <vector>

#include "huron/locomotion/zero_moment_point.h"

namespace huron {

template <typename T>
class ZeroMomentPointTotal : public ZeroMomentPoint<T> {
 public:
  ZeroMomentPointTotal(
    std::weak_ptr<const multibody::Frame<T>> zmp_frame,
    const std::vector<std::shared_ptr<ZeroMomentPoint<T>>>& zmp_vector);
  ZeroMomentPointTotal(const ZeroMomentPointTotal&) = delete;
  ZeroMomentPointTotal& operator=(const ZeroMomentPointTotal&) = delete;
  ~ZeroMomentPointTotal() override = default;

  Eigen::Vector2d Eval(double& fz) override;

 private:
  std::vector<std::shared_ptr<ZeroMomentPoint<T>>> zmp_vector_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPointTotal)
