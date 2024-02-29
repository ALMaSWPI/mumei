#pragma once

#include <string>
#include <memory>

#include "huron/multibody/frame.h"

namespace huron {
namespace multibody {

/**
 * @brief Robot center of mass frame.
 */
template <typename T>
class ComFrame : public Frame<T> {
 public:
  ComFrame(FrameIndex index,
           const std::string& name,
           bool is_user_defined,
           std::weak_ptr<const Model<T>> model,
           FrameIndex parent_frame_index);

  ComFrame(const ComFrame&) = delete;
  ComFrame& operator=(const ComFrame&) = delete;
  ~ComFrame() override = default;

  huron::SE3<T> GetTransformInWorld() const override;
  huron::SE3<T> GetTransformFromFrame(const Frame<T>& other) const override;
  huron::SE3<T> GetTransformFromFrame(FrameIndex other) const override;
  huron::SE3<T> GetTransformToFrame(const Frame<T>& other) const override;
  huron::SE3<T> GetTransformToFrame(FrameIndex other) const override;

 private:
  FrameIndex parent_frame_index_;

  huron::SE3<T> ParentToThisTransform() const;
};

}  // namespace multibody
}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::ComFrame)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::ComFrame)
