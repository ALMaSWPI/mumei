#pragma once

#include <string>
#include <memory>

#include "huron/multibody/fwd.h"
#include "huron/multibody/frame.h"

namespace huron {
namespace multibody {

/**
 * @brief A frame that is defined relative to another frame by an affine
 * transformation. This transformation is user-defined using a function that
 * takes the parent frame's transform in world coordinates as an argument and
 * returns the transform from the parent frame to this frame.
 *
 * @note This class can only be instantiated by the Model class using
 * AddLogicalFrame().
 *
 * @param index The index of this frame.
 * @param name The name of this frame.
 * @param model The model that this frame is a part of.
 * @param parent_frame_index The index of the frame that this frame is defined
 * relative to.
 * @param transform_function The function that defines the transformation from
 * the parent frame to this frame.
 */
template <typename T>
class LogicalFrame
  : public Frame<T>,
    enable_protected_make_shared<LogicalFrame<T>> {
 public:
  friend class Model<T>;

  LogicalFrame(const LogicalFrame&) = delete;
  LogicalFrame& operator=(const LogicalFrame&) = delete;
  ~LogicalFrame() override = default;

  huron::SE3<T> GetTransformInWorld() const override;
  huron::SE3<T> GetTransformFromFrame(const Frame<T>& other) const override;
  huron::SE3<T> GetTransformFromFrame(FrameIndex other) const override;
  huron::SE3<T> GetTransformToFrame(const Frame<T>& other) const override;
  huron::SE3<T> GetTransformToFrame(FrameIndex other) const override;

 protected:
  LogicalFrame(FrameIndex index,
               const std::string& name,
               bool is_user_defined,
               std::weak_ptr<const Model<T>> model,
               FrameIndex parent_frame_index,
               std::function<huron::SE3<T>(const huron::SE3<T>&)>
                 transform_function);

 private:
  FrameIndex parent_frame_index_;
  const std::function<huron::SE3<T>(const huron::SE3<T>&)>
    transform_function_;
};

}  // namespace multibody
}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::LogicalFrame)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::LogicalFrame)
