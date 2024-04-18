#include "huron/multibody/logical_frame.h"
#include "huron/multibody/model.h"

namespace huron {
namespace multibody {

template <typename T>
LogicalFrame<T>::LogicalFrame(
  FrameIndex index,
  const std::string& name,
  bool is_user_defined,
  std::weak_ptr<const Model<T>> model,
  FrameIndex parent_frame_index,
  std::function<huron::SE3<T>(const huron::SE3<T>&)> transform_function)
  : Frame<T>(index, name, FrameType::kLogical, is_user_defined, std::move(model)),
    parent_frame_index_(parent_frame_index),
    transform_function_(transform_function) {}

template <typename T>
huron::SE3<T> LogicalFrame<T>::GetTransformInWorld() const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransformInWorld(parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

template <typename T>
huron::SE3<T> LogicalFrame<T>::GetTransformFromFrame(const Frame<T>& other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(other.index(), parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

template <typename T>
huron::SE3<T> LogicalFrame<T>::GetTransformFromFrame(FrameIndex other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(other, parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

template <typename T>
huron::SE3<T> LogicalFrame<T>::GetTransformToFrame(const Frame<T>& other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(parent_frame_index_, other.index());
  return transform_function_(parent_transform).Inverse() * parent_transform;
}

template <typename T>
huron::SE3<T> LogicalFrame<T>::GetTransformToFrame(FrameIndex other) const {
  huron::SE3<T> parent_transform =
    this->model_.lock()->GetFrameTransform(parent_frame_index_, other);
  return transform_function_(parent_transform).Inverse() * parent_transform;
}

}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::LogicalFrame)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::LogicalFrame)
