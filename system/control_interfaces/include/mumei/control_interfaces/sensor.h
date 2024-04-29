#pragma once

#include <eigen3/Eigen/Core>

#include <memory>
#include <string>

#include "mumei/control_interfaces/generic_component.h"
#include "mumei/control_interfaces/state_provider.h"

namespace mumei {

class Sensor : public GenericComponent, public StateProvider {
 public:
  Sensor(const std::string& name,
         const Eigen::Vector2i& dim,
         std::unique_ptr<Configuration> config);
  Sensor(const std::string& name, const Eigen::Vector2i& dim);
  Sensor(const std::string& name,
         int rows, int cols,
         std::unique_ptr<Configuration> config);
  Sensor(const std::string& name,
         int rows, int cols);
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;
  virtual ~Sensor() = default;

  /**
   * @brief Get the sensor value.
   */
  virtual Eigen::VectorXd GetValue() const;
  virtual Eigen::VectorXd ReloadAndGetValue();
};

}  // namespace mumei
