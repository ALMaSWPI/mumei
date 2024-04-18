#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include "huron/control_interfaces/sensor_with_frame.h"
#include "huron/types.h"

namespace huron {

template <typename T>
class ForceTorqueSensor : public SensorWithFrame<T> {
 public:
  ForceTorqueSensor(bool reverse_wrench_direction,
                    std::weak_ptr<const multibody::Frame<T>> frame);
  ForceTorqueSensor(bool reverse_wrench_direction,
                    std::weak_ptr<const multibody::Frame<T>> frame,
                    std::unique_ptr<Configuration> config);
  ForceTorqueSensor(const ForceTorqueSensor&) = delete;
  ForceTorqueSensor& operator=(const ForceTorqueSensor&) = delete;
  ~ForceTorqueSensor() override = default;

  void RequestStateUpdate() override;

  void GetNewState(Eigen::Ref<huron::MatrixX<T>> new_state) const override;
  /**
   * Measures the external forces and moments.
   *
   * @return Wrench 6x1 vector \f$ [Fx, Fy, Fz, Tx, Ty, Tz]^T \f$.
   */
  huron::VectorX<T> GetValue() const override;

 protected:
  /**
   * To be overriden.
   */
  virtual huron::Vector6<T> DoGetWrenchRaw() = 0;

  bool reverse_wrench_direction_;

 private:
  huron::Vector6<T> wrench_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceTorqueSensor)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ForceTorqueSensor)
