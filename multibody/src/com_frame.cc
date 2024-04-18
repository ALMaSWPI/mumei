#include "huron/multibody/com_frame.h"
#include "huron/multibody/model.h"

namespace huron {
namespace multibody {

template <typename T>
ComFrame<T>::ComFrame(
  FrameIndex index,
  const std::string& name,
  bool is_user_defined,
  std::weak_ptr<const Model<T>> model,
  FrameIndex parent_frame_index)
  : Frame<T>(index, name, FrameType::kLogical, is_user_defined, std::move(model)),
    parent_frame_index_(parent_frame_index) {}

template <typename T>
huron::SE3<T> ComFrame<T>::GetTransformInWorld() const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransformInWorld(parent_frame_index_);
  return parent_transform * ParentToThisTransform();
}

template <typename T>
huron::SE3<T> ComFrame<T>::GetTransformFromFrame(const Frame<T>& other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(other.index(), parent_frame_index_);
  return parent_transform * ParentToThisTransform();
}

template <typename T>
huron::SE3<T> ComFrame<T>::GetTransformFromFrame(FrameIndex other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(other, parent_frame_index_);
  return parent_transform * ParentToThisTransform();
}

template <typename T>
huron::SE3<T> ComFrame<T>::GetTransformToFrame(const Frame<T>& other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(parent_frame_index_, other.index());
  return ParentToThisTransform().inverse() * parent_transform;
}

template <typename T>
huron::SE3<T> ComFrame<T>::GetTransformToFrame(FrameIndex other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(parent_frame_index_, other);
  return ParentToThisTransform().inverse() * parent_transform;
}

template <typename T>
huron::SE3<T> ComFrame<T>::ParentToThisTransform() const {
  huron::SE3<T> parent_to_this;
  parent_to_this.translate(this->model_.lock()->GetCenterOfMassPosition());
  return parent_to_this;
}

}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::ComFrame)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::ComFrame)
