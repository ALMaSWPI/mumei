#pragma once
#include "generic_component.h"

namespace huron {

/**
 * Abstract class for encoder
 * A generic encoder has count and velocity.
 *
 * @ingroup control_interface
 */
class Encoder : public GenericComponent {
 public:
  class EncoderConfiguration : public Configuration {
   private:
    static const inline std::set<std::string> kEncoderValidKeys{};

   public:
    EncoderConfiguration(ConfigMap config_map,
                         std::set<std::string> valid_keys)
        : Configuration(config_map, [&valid_keys]() {
                          std::set<std::string> tmp(kEncoderValidKeys);
                          tmp.merge(valid_keys);
                          return tmp;
                        }()) {}

    explicit EncoderConfiguration(ConfigMap config_map)
        : EncoderConfiguration(config_map, {}) {}

    EncoderConfiguration()
        : EncoderConfiguration({}, {}) {}
  };

  explicit Encoder(std::unique_ptr<EncoderConfiguration> config)
    : GenericComponent(std::move(config)) {}
  Encoder() : Encoder(std::make_unique<EncoderConfiguration>()) {}
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  virtual ~Encoder() = default;

  virtual float GetPosition() = 0;
  virtual float GetVelocity() = 0;
  virtual void Reset() = 0;

  /**
   * Gets a reference to the underlying driver.
   */
  virtual GenericComponent& GetDriver() = 0;
};

}  // namespace huron
