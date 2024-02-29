#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include "robot.h"
#include "huron/locomotion/zero_moment_point.h"

namespace huron {

template <typename T>
class LeggedRobot : public Robot<T> {
 public:
  explicit LeggedRobot(std::unique_ptr<RobotConfiguration> config);
  LeggedRobot();
  LeggedRobot(const LeggedRobot&) = delete;
  LeggedRobot& operator=(const LeggedRobot&) = delete;
  ~LeggedRobot() override = default;

  void InitializeZmp(std::shared_ptr<ZeroMomentPoint<T>> zmp);
  /**
   * Computes the Center of Mass in Base frame.
   */
  huron::Vector2<T> EvalZeroMomentPoint();

 private:
  std::shared_ptr<ZeroMomentPoint<T>> zmp_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::LeggedRobot)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::LeggedRobot)
