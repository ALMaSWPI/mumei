#pragma once

#include <vector>
#include <string>
#include <memory>

#include "mumei/control_interfaces/motor.h"
#include "mumei/enable_protected_make_shared.h"

namespace mumei {
namespace mujoco {

class MujocoEnvironment;

class Motor : public mumei::Motor, public enable_protected_make_shared<Motor> {
  friend class MujocoEnvironment;
 public:
  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;
  ~Motor() = default;

  // GenericComponent methods
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  bool Move(const std::vector<double>& values) override;
  bool Move(const Eigen::VectorXd& values) override;
  bool Stop() override;

  bool Move(double value) override;

 protected:
  Motor(const std::string& name,
        int mj_id,
        std::weak_ptr<MujocoEnvironment> env,
        double gear_ratio = 1.0);

 private:
  int mj_id_;
  std::weak_ptr<MujocoEnvironment> env_;
};

}  // namespace mujoco
}  // namespace mumei
