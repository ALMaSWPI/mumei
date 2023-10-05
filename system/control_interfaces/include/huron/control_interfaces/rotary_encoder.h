#pragma once
#include <cmath>
#include "encoder.h"

namespace huron {
/**
 * Abstract class for using an encoder.
 *
 * @ingroup control_interfaces
 */
class RotaryEncoder : public Encoder {
 protected:
  float velocity_ = 0;
  float prev_velocity_ = 0;
  float count_ = 0;
  float prev_count_ = 0;
  float cpr_;

 public:
  class RotaryEncoderConfiguration : public EncoderConfiguration {
   public:
    /**
     * Supports further inheritance.
     */
    RotaryEncoderConfiguration(ConfigMap config_map,
                               std::set<std::string> valid_keys)
      : EncoderConfiguration(config_map,
                             [&valid_keys]() {
                               std::set<std::string> tmp(kRotEncValidKeys);
                               tmp.merge(valid_keys);
                               return tmp;
                             }()) {}

    RotaryEncoderConfiguration(float cpr)
      : RotaryEncoderConfiguration(
          ConfigMap({{"cpr", cpr}}), {}) {}

   private:
    static const inline std::set<std::string> kRotEncValidKeys{"cpr"};
  };

  explicit RotaryEncoder(std::unique_ptr<RotaryEncoderConfiguration> config)
      : Encoder(std::move(config)) {
    cpr_ = std::any_cast<float>(config_.get()->Get("cpr"));
  }
  explicit RotaryEncoder(float cpr)
      : RotaryEncoder(std::make_unique<RotaryEncoderConfiguration>(cpr)) {}
  RotaryEncoder(const RotaryEncoder&) = delete;
  RotaryEncoder& operator=(const RotaryEncoder&) = delete;
  ~RotaryEncoder() override = default;

  /**
     * Gets the current encoder count.
     */
  virtual float GetCount() = 0;

  /**
     * Gets the current encoder velocity in count.
     */
  virtual float GetVelocityCount() = 0;

  /**
     * Gets the previous encoder count.
     */
  float GetPrevCount() {
    return prev_count_;
  }

  /**
     * Gets the counts per revolution (CPR).
     */
  float GetCPR() {
    return cpr_;
  }

  /**
     * Gets the current angle in radians.
     */
  float GetPosition() override {
    return GetCount() / cpr_ * 2.0 * M_PI;
  }

  /**
     * Gets the current angle in degrees.
     */
  float GetAngleDegree() {
    return GetCount() / cpr_ * 360.0;
  }

  /**
     * Gets the current velocity in radians/second.
     */
  float GetVelocity() override {
    return GetVelocityCount() / cpr_ * 2 * M_PI;
  }

  /**
     * Gets the current velocity in degrees/second.
     */
  float GetVelocityDegree() {
    return GetVelocityCount() / cpr_ * 360.0;
  }

  /**
     * Resets the encoder count.
     */
  void Reset() override {
    count_ = 0.0;
    prev_count_ = 0.0;
  }
};

}  // namespace huron
