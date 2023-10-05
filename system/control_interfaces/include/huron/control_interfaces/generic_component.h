#pragma once

#include <memory>
#include <unordered_map>
#include "configuration.h"

namespace huron {

/**
 * Interface for all components.
 *
 * @ingroup control_interfaces
 */
class GenericComponent {
 protected:
  std::unique_ptr<Configuration> config_;

  /**
   * Configure the hardware component with the specified key-value pair. This
   * method needs to be defined by the user.
   *
   * @pre The configuration pair is valid.
   */
  virtual void ConfigureKey(std::string config_key, std::any config_value) = 0;

  /**
   * Configure the hardware component with the specified configuration map.
   *
   * @pre The configuration map is valid.
   */
  virtual void ConfigureMap(const ConfigMap& config_map) {
    for (auto& pair : config_map) {
      ConfigureKey(pair.first, pair.second);
    }
  }

 public:
  explicit GenericComponent(std::unique_ptr<Configuration> config)
    : config_(std::move(config)) {}
  GenericComponent(const GenericComponent&) = delete;
  GenericComponent& operator=(const GenericComponent&) = delete;
  virtual ~GenericComponent() = default;

  void Configure(std::string config_key, std::any config_value) {
    if (config_->Set(config_key, config_value)) {
      return ConfigureKey(config_key, config_value);
    }
  }
  void Configure(ConfigMap config) {
    if (config_->Set(config)) {
      return ConfigureMap(config);
    }
  }
  void Configure(std::unique_ptr<Configuration> config_ptr) {
    config_ = std::move(config_ptr);
  }

  virtual void Initialize() = 0;
  virtual void SetUp() = 0;
  virtual void Terminate() = 0;
};

}  // namespace huron
