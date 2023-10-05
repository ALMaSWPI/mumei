#pragma once

#include <memory>
#include <vector>

#include "motor.h"
#include "moving_component.h"
#include "encoder.h"

namespace huron {

/**
 * Abstract class representing a joint.
 *
 * Currently, the structure supports 1-DoF joints.
 *
 * @ingroup control_interfaces
 */
class Joint : public MovingComponent {
 public:
  class JointConfiguration : public Configuration {
   private:
    static const inline std::set<std::string> kJointValidKeys{};

   public:
    JointConfiguration(ConfigMap config_map, std::set<std::string> valid_keys)
        : Configuration(config_map,
                        [&valid_keys]() {
                          std::set<std::string> tmp(kJointValidKeys);
                          tmp.merge(valid_keys);
                          return tmp;
                        }()) {}

    JointConfiguration(ConfigMap config_map)
        : JointConfiguration(config_map, {}) {}

    JointConfiguration()
        : JointConfiguration({}, {}) {}

  };

  Joint(std::unique_ptr<Motor> motor, std::unique_ptr<Encoder> encoder,
        std::unique_ptr<JointConfiguration> config);
  Joint(std::unique_ptr<Motor> motor, std::unique_ptr<Encoder> encoder);
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;
  virtual ~Joint() = default;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  bool Move(float value) override;
  bool Move(const std::vector<float>& value) override;
  bool Stop() override;

  /**
   * Gets the position of the joint.
   */
  virtual float GetPosition() = 0;
  /**
   * Gets the velocity of the joint.
   */
  virtual float GetVelocity() = 0;
  /**
   * Gets the acceleration of the joint.
   */
  virtual float GetAcceleration() = 0;

 protected:
  std::unique_ptr<Motor> motor_;
  std::unique_ptr<Encoder> encoder_;
};

}  // namespace huron
