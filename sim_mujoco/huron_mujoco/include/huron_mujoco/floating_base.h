#pragma once

#include <string>
#include <memory>

#include "huron/control_interfaces/state_provider.h"
#include "huron/enable_protected_make_shared.h"

namespace huron {
namespace mujoco {

class MujocoEnvironment;

class FloatingBase : public huron::StateProvider,
                     public enable_protected_make_shared<FloatingBase> {
  friend class MujocoEnvironment;
 public:
  FloatingBase(const FloatingBase&) = delete;
  FloatingBase& operator=(const FloatingBase&) = delete;
  ~FloatingBase() = default;

  void RequestStateUpdate() override;
  void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const override;

 protected:
  FloatingBase(const std::string& name,
               int id,
               std::weak_ptr<MujocoEnvironment> env);

 private:
  std::string name_;
  int id_;
  std::weak_ptr<MujocoEnvironment> env_;

  Eigen::Vector<double, 13> states_;
};

}  // namespace mujoco
}  // namespace huron
