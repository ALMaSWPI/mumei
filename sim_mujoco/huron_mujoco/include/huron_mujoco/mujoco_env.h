#include "huron/environment.h"
#include "huron/control_interfaces/motor.h"
#include "huron/control_interfaces/encoder.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

namespace huron {
namespace mujoco {

class MujocoEnvironment
  : public Environment,
    public std::enable_shared_from_this<MujocoEnvironment>{
 public:
  MujocoEnvironment(std::function<void(void)> loop_func,
                    std::function<void(void)> exit_func = []() {});
  MujocoEnvironment(const MujocoEnvironment&) = delete;
  MujocoEnvironment& operator=(const MujocoEnvironment&) = delete;
  ~MujocoEnvironment() = default;

  void Initialize(int argc, char* argv[]) override;
  void Configure(void* config) override;
  void Finalize() override;
  void Exit() override;

  std::shared_ptr<huron::Motor> CreateMotor(
    const std::string& name,
    double gear_ratio = 1.0);

  std::shared_ptr<huron::Motor> CreateMotor(
    int id,
    double gear_ratio = 1.0);

  std::shared_ptr<huron::Encoder> CreateEncoder(
    const std::string& name,
    double gear_ratio = 1.0);

  std::shared_ptr<huron::StateProvider> CreateFloatingBase(
    const std::string& name = "");

  Eigen::Vector2d GetEncoderValues(int id) const;
  Eigen::Vector<double, 13> GetFloatingBaseStates(int id) const;
  void SetMotorValue(int id, double u);

  void PrintStates() const;

 protected:
  void LoopPrologue() override;
  void LoopEpilogue() override;
};


}  // namespace ros2
}  // namespace huron
