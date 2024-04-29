#pragma once

#include <vector>
#include <set>
#include <string>
#include <utility>
#include <memory>
#include <unordered_map>

#include "mumei/control_interfaces/configuration.h"
#include "mumei/control_interfaces/generic_component.h"
#include "mumei/control_interfaces/moving_group.h"
#include "mumei/control_interfaces/joint.h"
#include "mumei/multibody/model.h"
#include "mumei/multibody/joint_common.h"

namespace mumei {

class RobotConfiguration : public Configuration {
 private:
  static const inline std::set<std::string> kRobotValidKeys{};

 public:
  RobotConfiguration(ConfigMap config_map,
                     std::set<std::string> valid_keys)
      : Configuration(config_map, [&valid_keys]() {
                        std::set<std::string> tmp(kRobotValidKeys);
                        tmp.merge(valid_keys);
                        return tmp;
                      }()) {}

  explicit RobotConfiguration(ConfigMap config_map)
      : RobotConfiguration(config_map, {}) {}

  RobotConfiguration()
      : RobotConfiguration({}, {}) {}
};

class Robot : public MovingGroup, public GenericComponent {
  using Model = multibody::Model;

 public:
  explicit Robot(std::unique_ptr<RobotConfiguration> config);
  Robot();
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  ~Robot() override = default;

  // GenericComponent
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  Model* const GetModel() { return model_.get(); }

  void RegisterStateProvider(std::shared_ptr<StateProvider> state_provider,
                             bool is_joint_state_provider = false);

  /**
   * Calls RequestStateUpdate() on all the registered state providers.
   */
  void UpdateAllStates();

  /**
   * Calls RequestStateUpdate() on all the registered state providers for
   * joints.
   */
  void UpdateJointStates();

  const Eigen::VectorBlock<const Eigen::VectorXd> GetJointPositions() const;

  const Eigen::VectorBlock<const Eigen::VectorXd> GetJointVelocities() const;

  std::weak_ptr<Indexable> GetComponent(const std::string& name) const;
  std::weak_ptr<Indexable> GetComponent(Index id) const;

 protected:
  Robot(std::unique_ptr<RobotConfiguration> config,
        std::shared_ptr<Model> model);
  explicit Robot(std::shared_ptr<Model> model);

  std::shared_ptr<Model> model_;
  std::vector<std::shared_ptr<StateProvider>> non_joint_state_providers_;
  std::vector<std::shared_ptr<StateProvider>> joint_state_providers_;

  std::unordered_map<std::string, Index> name_to_index_;
  std::vector<std::weak_ptr<Indexable>> registered_components_;
};

}  // namespace mumei
