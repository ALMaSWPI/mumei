#include <iostream>

#include "huron/control_interfaces/configuration.h"
#include "huron/exceptions/invalid_configuration_exception.h"

namespace huron {

Configuration::Configuration(ConfigMap config_map,
                             std::set<std::string> valid_keys)
  : valid_keys_(valid_keys),
    config_map_(ValidateMap(config_map)) {}

Configuration::Configuration(ConfigMap config_map)
  : Configuration(config_map, std::set<std::string>()) {}

ConfigMap Configuration::ValidateMap(ConfigMap config_map) {
  for (auto& pair : config_map) {
    if (!ValidateKey(pair.first)) {
      throw InvalidConfigurationException();
    }
  }
  return config_map;
}

std::any Configuration::Get(std::string config_key, bool renew) {
  if (!ValidateKey(config_key)) {
    throw InvalidConfigurationException();
  }
  ConfigMap::iterator it;
  if (renew || (it= config_map_.find(config_key)) == config_map_.end()) {
    config_map_.emplace(config_key, GetFromComponent(config_key));
  }
  return config_map_[config_key];
}

bool Configuration::Set(std::string config_key, std::any config_value) {
  if (ValidateKey(config_key)) {
    config_map_[config_key] = config_value;

    std::cout << "ODrive config:\n";
    for (auto& pair : config_map_) {
      std::cout << pair.first << " : " 
                << std::any_cast<float>(pair.second) << std::endl;
    }
    std::cout << std::endl;

    return true;
  }
  return false;
}

// Validates all key-value pairs first.
// Only apply configuration if all pairs are valid.
bool Configuration::Set(ConfigMap config_map) {
  for (auto& pair : config_map) {
    if (!ValidateKey(pair.first)) {
      return false;
    }
  }
  for (auto& pair : config_map) {
    Set(pair.first, pair.second);
  }
  return true;
}

}  // namespace huron
