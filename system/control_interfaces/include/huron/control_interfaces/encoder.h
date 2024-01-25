#pragma once

#include <set>
#include <string>
#include <utility>
#include <memory>

#include "huron/control_interfaces/sensor.h"

namespace huron {

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

/**
 * Abstract class for encoder
 * A generic encoder has count and velocity.
 *
 * @ingroup control_interface
 */
class Encoder : public Sensor {
 public:
  Encoder(double gear_ratio, std::unique_ptr<EncoderConfiguration> config)
    : Sensor(2, 1, std::move(config)), gear_ratio_(gear_ratio) {}
  explicit Encoder(double gear_ratio)
    : Encoder(gear_ratio, std::make_unique<EncoderConfiguration>()) {}
  explicit Encoder(std::unique_ptr<EncoderConfiguration> config)
    : Encoder(1.0, std::move(config)) {}
  Encoder() : Encoder(1.0) {}
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  virtual ~Encoder() = default;

  void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const override {
    new_state = Eigen::Vector2d(GetPosition(), GetVelocity());
  }

  virtual double GetPosition() const = 0;
  virtual double GetVelocity() const = 0;

  virtual void Reset() = 0;

 protected:
  double gear_ratio_;
};

}  // namespace huron
