#include "mumei/control_interfaces/sensor.h"

namespace mumei {

Sensor::Sensor(const std::string& name,
               const Eigen::Vector2i& dim,
               std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)),
      StateProvider(name, dim) {}

Sensor::Sensor(const std::string& name,
               const Eigen::Vector2i& dim)
  : GenericComponent(),
    StateProvider(name, dim) {}

Sensor::Sensor(const std::string& name,
               int rows, int cols,
               std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)),
      StateProvider(name, rows, cols) {}

Sensor::Sensor(const std::string& name, int rows, int cols)
  : GenericComponent(),
    StateProvider(name, rows, cols) {}

Eigen::VectorXd Sensor::GetValue() const {
  Eigen::VectorXd tmp;
  GetNewState(tmp);
  return tmp;
}

Eigen::VectorXd Sensor::ReloadAndGetValue() {
  RequestStateUpdate();
  return GetValue();
}

}  // namespace mumei
