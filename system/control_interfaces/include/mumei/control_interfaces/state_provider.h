#pragma once

#include <eigen3/Eigen/Dense>
#include <string>
#include "mumei/control_interfaces/indexable.h"

namespace mumei {

class StateProvider : public Indexable {
 public:
  StateProvider(const std::string& name, const Eigen::Vector2i& dim)
    : Indexable(name), dim_(dim) {}
  StateProvider(const std::string& name, int rows, int cols)
    : Indexable(name), dim_(rows, cols) {}
  StateProvider(const StateProvider&) = delete;
  StateProvider& operator=(const StateProvider&) = delete;
  virtual ~StateProvider() = default;

  virtual void RequestStateUpdate() = 0;
  virtual void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const = 0;

  const Eigen::Vector2i& dim() const { return dim_; }

 private:
  const Eigen::Vector2i dim_;
};

}  // namespace mumei
