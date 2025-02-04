#pragma once

#include <vector>
#include <set>
#include <string>
#include <utility>
#include <memory>

#include "mumei/control_interfaces/moving_interface.h"
#include "mumei/control_interfaces/generic_component.h"
#include "mumei/control_interfaces/indexable.h"

namespace mumei {

class ActuatorConfiguration : public Configuration {
 private:
  static const inline std::set<std::string> kActuatorValidKeys{};

 public:
  ActuatorConfiguration(ConfigMap config_map,
                     std::set<std::string> valid_keys)
      : Configuration(config_map, [&valid_keys]() {
                        std::set<std::string> tmp(kActuatorValidKeys);
                        tmp.merge(valid_keys);
                        return tmp;
                      }()) {}

  explicit ActuatorConfiguration(ConfigMap config_map)
      : ActuatorConfiguration(config_map, {}) {}

  ActuatorConfiguration()
      : ActuatorConfiguration({}, {}) {}
};

class Actuator : public GenericComponent,
                 public MovingInterface,
                 public Indexable {
 public:
  Actuator(const std::string& name,
           size_t dim,
           std::unique_ptr<ActuatorConfiguration> config)
    : GenericComponent(std::move(config)),
      MovingInterface(dim),
      Indexable(name) {}
  Actuator(const std::string& name, size_t dim)
    : Actuator(name, dim, std::make_unique<ActuatorConfiguration>()) {}
  Actuator(const Actuator&) = delete;
  Actuator& operator=(const Actuator&) = delete;
  ~Actuator() override = default;
};

}  // namespace mumei
