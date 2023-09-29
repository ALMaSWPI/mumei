#pragma once

#include <string>
#include <unordered_map>
#include <any>

namespace huron {

typedef std::unordered_map<std::any, std::any> ConfigMap;

/**
 * Abstract data structure for component configuration.
 *
 * @ingroup control_interface
 */
class Configuration {
 protected:
  ConfigMap config_map_;

  virtual bool ValidateKey(std::any config_key, std::any config_value);
  ConfigMap ValidateMap(ConfigMap config_map);

 public:
  explicit Configuration(const ConfigMap& config_map);
  explicit Configuration(ConfigMap config_map);
  Configuration(const Configuration&) = delete;
  Configuration& operator=(const Configuration&) = delete;
  virtual ~Configuration() = default;

  virtual std::any Get(std::any config_key) const = 0;
  bool Set(std::any config_key, std::any config_value);
  bool Set(ConfigMap config_map);
};

}  // namespace huron
