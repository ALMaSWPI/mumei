#pragma once

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <memory>

#include "encoder.h"

namespace huron {

template <typename T>
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

  explicit RotaryEncoderConfiguration(T cpr)
      : RotaryEncoderConfiguration(
          ConfigMap({{"cpr", cpr}}), {}) {}

 private:
  static const inline std::set<std::string> kRotEncValidKeys{"cpr"};
};

/**
 * Abstract class for using an encoder.
 *
 * @ingroup control_interfaces
 */
template <typename T>
class RotaryEncoder : public Encoder<T> {
 public:
  RotaryEncoder(T gear_ratio,
                std::unique_ptr<RotaryEncoderConfiguration<T>> config)
    : Encoder<T>(gear_ratio, std::move(config)) {
    cpr_ = std::any_cast<T>(this->config_.get()->Get("cpr"));
  }
  RotaryEncoder(T gear_ratio, T cpr)
      : RotaryEncoder(gear_ratio,
                      std::make_unique<RotaryEncoderConfiguration<T>>(cpr)) {}
  RotaryEncoder(const RotaryEncoder&) = delete;
  RotaryEncoder& operator=(const RotaryEncoder&) = delete;
  ~RotaryEncoder() override = default;

  void RequestStateUpdate() final {
    prev_count_ = count_;
    prev_velocity_ = velocity_;
    DoUpdateState();
  }

  /**
     * Gets the current encoder count.
     */
  T GetCount() const {
    return count_;
  }

  /**
     * Gets the current encoder velocity in count.
     */
  T GetVelocityCount() const {
    return velocity_;
  }

  /**
     * Gets the previous encoder count.
     */
  T GetPrevCount() const {
    return prev_count_;
  }

  /**
     * Gets the counts per revolution (CPR).
     */
  T GetCPR() const {
    return cpr_;
  }

  /**
     * Gets the current angle in radians. This takes into account the gear ratio
     * and CPR.
     */
  T GetPosition() const override {
    return count_ / cpr_ * 2.0 * M_PI / this->gear_ratio_;
  }

  /**
     * Gets the current angle in degrees. This takes into account the gear ratio
     * and CPR.
     */
  T GetAngleDegree() const {
    return count_ / cpr_ * 360.0 / this->gear_ratio_;
  }

  /**
     * Gets the current velocity in radians/second. This takes into account the
     * gear ratio and CPR.
     */
  T GetVelocity() const override {
    return velocity_ / cpr_ * 2 * M_PI / this->gear_ratio_;
  }

  /**
     * Gets the current velocity in degrees/second. This takes into account the
     * gear ratio and CPR.
     */
  T GetVelocityDegree() const {
    return velocity_ / cpr_ * 360.0 / this->gear_ratio_;
  }

  /**
     * Resets the encoder count.
     */
  void Reset() override {
    count_ = 0.0;
    prev_count_ = 0.0;
  }

 protected:
  /**
   * Classes derived from RotaryEncoder should override this function instead
   * of directly overriding RequestUpdateState(). RotaryEncoder already handled
   * the internal count update for convenience.
   *
   * This function should update the current count (count_) and, if possible,
   * velocity (velocity_).
   */
  virtual void DoUpdateState() = 0;

  /// \brief Encoder velocity in counts per second.
  T velocity_;
  /// \brief Encoder previous velocity in counts per second.
  T prev_velocity_;
  T count_;
  T prev_count_;
  T cpr_;
};

}  // namespace huron

// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class huron::RotaryEncoderConfiguration)
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
//     class huron::RotaryEncoderConfiguration)
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class huron::RotaryEncoder)
// HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
//     class huron::RotaryEncoder)
