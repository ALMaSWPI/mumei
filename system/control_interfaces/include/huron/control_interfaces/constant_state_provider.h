#pragma once

#include "state_provider.h"

namespace huron {

template <typename T>
class ConstantStateProvider : public StateProvider<T> {
 public:
  explicit ConstantStateProvider(const huron::MatrixX<T>& state)
    : StateProvider<T>(state.rows(), state.cols()),
      state_(state) {}
  ConstantStateProvider(const ConstantStateProvider&) = delete;
  ConstantStateProvider& operator=(const ConstantStateProvider&) = delete;
  ~ConstantStateProvider() override = default;

  void RequestStateUpdate() override {}
  void GetNewState(Eigen::Ref<huron::MatrixX<T>> new_state) const override {
    new_state = state_;
  }

  void SetState(const huron::MatrixX<T>& state) {
    assert(state.rows() == dim()[0] && state.cols() == dim()[1]);
    state_ = state;
  }

 private:
  huron::MatrixX<T> state_;
};

}  // namespace huron
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class huron::ConstantStateProvider)
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
//     class huron::ConstantStateProvider)
