#pragma once

#include "huron/control_interfaces/motor.h"
#include "huron/enable_protected_make_shared.h"

namespace huron {
namespace mujoco {

class MujocoEnvironment;

class Motor : public huron::Motor, public enable_protected_make_shared<Motor> {
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
  Motor(const std::string& name, int id, std::weak_ptr<MujocoEnvironment> env,
        double gear_ratio = 1.0);

 private:
  std::string name_;
  int id_;
  std::weak_ptr<MujocoEnvironment> env_;
};

}  // namespace mujoco
}  // namespace huron
