#pragma once

#include <string>
#include <memory>

#include "mumei/control_interfaces/state_provider.h"
#include "mumei/enable_protected_make_shared.h"

namespace mumei {
namespace mujoco {

class MujocoEnvironment;

class FloatingBase : public mumei::StateProvider,
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
               int mj_id,
               std::weak_ptr<MujocoEnvironment> env);

 private:
  int mj_id_;
  std::weak_ptr<MujocoEnvironment> env_;

  Eigen::Vector<double, 13> states_;
};

}  // namespace mujoco
}  // namespace mumei
