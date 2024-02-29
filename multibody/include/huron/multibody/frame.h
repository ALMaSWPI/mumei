#pragma once

#include <eigen3/Eigen/Dense>

#include <memory>
#include <string>

#include "huron/multibody/fwd.h"
#include "huron/types.h"
#include "huron/math/se3.h"
#include "huron/utils/template_instantiations.h"
#include "huron/enable_protected_make_shared.h"

namespace huron {
namespace multibody {

using FrameIndex = size_t;

enum class FrameType {
  kLogical,
  kFixed,
  kJoint,
  kSensor,
  kPhysical,
};

template <typename T>
class Frame : public enable_protected_make_shared<Frame<T>> {
 public:
  friend class Model<T>;

  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;
  virtual ~Frame() = default;

  virtual huron::SE3<T> GetTransformInWorld() const;
  virtual huron::SE3<T> GetTransformFromFrame(const Frame& other) const;
  virtual huron::SE3<T> GetTransformFromFrame(FrameIndex other) const;
  virtual huron::SE3<T> GetTransformToFrame(const Frame& other) const;
  virtual huron::SE3<T> GetTransformToFrame(FrameIndex other) const;

  const std::string& name() const { return name_; }
  FrameIndex index() const { return index_; }
  FrameType type() const { return type_; }
  bool is_user_defined() const { return is_user_defined_; }

 protected:
  Frame(FrameIndex index,
        const std::string& name,
        FrameType type,
        bool is_user_defined,
        std::weak_ptr<const Model<T>> model);

  /// \brief Frame name.
  const FrameIndex index_;
  const std::string name_;
  const FrameType type_;
  bool is_user_defined_;
  const std::weak_ptr<const Model<T>> model_;
};

}  // namespace multibody
}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::Frame)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::Frame)
