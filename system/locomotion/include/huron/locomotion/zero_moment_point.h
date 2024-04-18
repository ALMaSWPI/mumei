#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include "huron/sensors/force_torque_sensor.h"
#include "huron/sensors/force_sensing_resistor_array.h"
#include "huron/math/se3.h"
#include "huron/multibody/logical_frame.h"

namespace huron {

template <typename T>
class ZeroMomentPoint {
 public:
  ZeroMomentPoint(std::weak_ptr<const multibody::Frame<T>> zmp_frame,
                  T normal_force_threshold);
  ZeroMomentPoint(const ZeroMomentPoint&) = delete;
  ZeroMomentPoint& operator=(const ZeroMomentPoint&) = delete;
  virtual ~ZeroMomentPoint() = default;

  /**
   * Evaluate the zero moment point in the ZMP frame based on the current
   * sensor and joint states.
   *
   * @param zmp The zero moment point in the ZMP frame.
   * @param fz The normal force.
   */
  virtual huron::Vector2<T> Eval(T& fz) = 0;
  huron::Vector2<T> Eval() {
    T fz;
    return Eval(fz);
  }

  /**
   * Convert the zero moment point from the 2D ZMP frame to the world frame.
   *
   * @param zmp The zero moment point in the ZMP frame.
   * @return The zero moment point in the world frame.
   */
  huron::SE3<T> ZmpToWorld(const huron::Vector2<T>& zmp) const;

 protected:
  std::weak_ptr<const multibody::Frame<T>> zmp_frame_;
  T normal_force_threshold_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ZeroMomentPoint)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ZeroMomentPoint)
