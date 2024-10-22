#pragma once

#include <eigen3/Eigen/Dense>

#include <memory>
#include <utility>
#include <vector>

#include "moving_interface.h"

namespace mumei {

class MovingGroup : public MovingInterface {
 public:
  MovingGroup();
  MovingGroup(const MovingGroup&) = delete;
  MovingGroup& operator=(const MovingGroup&) = delete;
  ~MovingGroup() override = default;

  virtual void AddToGroup(std::shared_ptr<MovingInterface> component);

  bool Move(const std::vector<double>& values) override;

  bool Move(const Eigen::VectorXd& values) override;

  bool Stop() override;

 protected:
  std::vector<std::shared_ptr<MovingInterface>> moving_components_;
  std::vector<size_t> moving_interface_dims_;
};

}  // namespace mumei
