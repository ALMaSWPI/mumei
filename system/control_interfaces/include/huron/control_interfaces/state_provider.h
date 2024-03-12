#pragma once

#include "huron/types.h"
#include "huron/utils/template_instantiations.h"

namespace huron {

template <typename T>
class StateProvider {
 public:
  explicit StateProvider(const Eigen::Vector2i& dim)
    : dim_(dim) {}
  StateProvider(int rows, int cols)
    : dim_(rows, cols) {}
  StateProvider(const StateProvider&) = delete;
  StateProvider& operator=(const StateProvider&) = delete;
  virtual ~StateProvider() = default;

  virtual void RequestStateUpdate() = 0;
  virtual void GetNewState(Eigen::Ref<huron::MatrixX<T>> new_state) const = 0;

  const Eigen::Vector2i& dim() const { return dim_; }

 private:
  const Eigen::Vector2i dim_;
};

}  // namespace huron

// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class huron::StateProvider)
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
//     class huron::StateProvider)
