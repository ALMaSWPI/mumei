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
template <typename T>
class Encoder : public Sensor<T> {
 public:
  Encoder(T gear_ratio, std::unique_ptr<EncoderConfiguration> config)
    : Sensor<T>(2, 1, std::move(config)), gear_ratio_(gear_ratio) {}
  explicit Encoder(T gear_ratio)
    : Encoder<T>(gear_ratio, std::make_unique<EncoderConfiguration>()) {}
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  virtual ~Encoder() = default;

  void GetNewState(Eigen::Ref<huron::MatrixX<T>> new_state) const override {
    new_state = huron::Vector2<T>(GetPosition(), GetVelocity());
  }

  virtual T GetPosition() const = 0;
  virtual T GetVelocity() const = 0;

  virtual void Reset() = 0;

 protected:
  T gear_ratio_;
};

}  // namespace huron

// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class huron::Encoder)
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
//     class huron::Encoder)
