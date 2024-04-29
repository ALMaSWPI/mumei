#pragma once

#include <string>
#include <memory>

#include "mumei/control_interfaces/encoder.h"
#include "mumei/enable_protected_make_shared.h"

namespace mumei {
namespace mujoco {

class MujocoEnvironment;

class Encoder : public mumei::Encoder,
                public enable_protected_make_shared<Encoder> {
  friend class MujocoEnvironment;
 public:
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  ~Encoder() = default;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  double GetPosition() const override;
  double GetVelocity() const override;

  void Reset() override;

  void RequestStateUpdate() override;


 protected:
  Encoder(const std::string& name,
          int mj_id,
          std::weak_ptr<MujocoEnvironment> env,
          double gear_ratio = 1.0);

 private:
  std::string name_;
  int mj_id_;
  std::weak_ptr<MujocoEnvironment> env_;

  double position_;
  double velocity_;
};

}  // namespace mujoco
}  // namespace mumei
