#pragma once

#include <string>
#include <unordered_map>
#include <set>
#include <any>

#include "huron/exceptions/not_implemented_exception.h"

namespace huron {

typedef std::unordered_map<std::string, std::any> ConfigMap;

/**
 * Abstract data structure for component configuration.
 *
 * @ingroup control_interface
 */
class Configuration {
 protected:
  const std::set<std::string> valid_keys_;
  ConfigMap config_map_;

  /**
   * Checks if the key is valid (i.e. in a list of valid keys).
   */
  bool ValidateKey(std::string config_key) {
    return valid_keys_.count(config_key);
  }
  ConfigMap ValidateMap(ConfigMap config_map);
  /**
   * Gets the configuration value from the hardware component. This method
   * needs to be overriden by concrete configuration classes.
   */
  virtual std::any GetFromComponent(std::string config_key) {
    throw NotImplementedException();
  }

 public:
  Configuration(ConfigMap config_map, std::set<std::string> valid_keys);
  explicit Configuration(ConfigMap config_map);
  Configuration(const Configuration&) = delete;
  Configuration& operator=(const Configuration&) = delete;
  virtual ~Configuration() = default;

  /**
   * Gets the value of the configuration with key @p config_key.
   *
   * If the configuration is not cached, gets the value from the component
   * (e.g. from the hardware), caches the value, then returns it. To force
   * getting a new value, set @p renew to true.
   *
   * @throw InvalidConfigurationException if @p config_key is invalid.
   */
  std::any Get(std::string config_key, bool renew=false);
  bool Set(std::string config_key, std::any config_value);
  bool Set(ConfigMap config_map);
};

}  // namespace huron
