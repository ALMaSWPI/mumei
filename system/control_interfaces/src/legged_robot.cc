#include "huron/control_interfaces/legged_robot.h"

namespace huron {

template <typename T>
LeggedRobot<T>::LeggedRobot(std::unique_ptr<RobotConfiguration> config)
  : Robot<T>(std::move(config)) {}

template <typename T>
LeggedRobot<T>::LeggedRobot()
  : Robot<T>() {}

template <typename T>
void LeggedRobot<T>::InitializeZmp(std::shared_ptr<ZeroMomentPoint<T>> zmp) {
  zmp_ = std::move(zmp);
}

template <typename T>
huron::Vector2<T> LeggedRobot<T>::EvalZeroMomentPoint() {
  return zmp_->Eval();
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::LeggedRobot)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::LeggedRobot)
