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
  std::function<Eigen::Affine3d(const Eigen::Affine3d&)> transform_function)
  : Frame<T>(index, name, FrameType::kLogical, is_user_defined, std::move(model)),
    parent_frame_index_(parent_frame_index),
    transform_function_(transform_function) {}

template <typename T>
Eigen::Affine3d LogicalFrame<T>::GetTransformInWorld() const {
  Eigen::Affine3d parent_transform =
    this->model_.lock()->GetFrameTransformInWorld(parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

template <typename T>
Eigen::Affine3d LogicalFrame<T>::GetTransformFromFrame(const Frame<T>& other) const {
  Eigen::Affine3d parent_transform =
    this->model_.lock()->GetFrameTransform(other.index(), parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

template <typename T>
Eigen::Affine3d LogicalFrame<T>::GetTransformFromFrame(FrameIndex other) const {
  Eigen::Affine3d parent_transform =
    this->model_.lock()->GetFrameTransform(other, parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

template <typename T>
Eigen::Affine3d LogicalFrame<T>::GetTransformToFrame(const Frame<T>& other) const {
  Eigen::Affine3d parent_transform =
    this->model_.lock()->GetFrameTransform(parent_frame_index_, other.index());
  return transform_function_(parent_transform).inverse() * parent_transform;
}

template <typename T>
Eigen::Affine3d LogicalFrame<T>::GetTransformToFrame(FrameIndex other) const {
  Eigen::Affine3d parent_transform =
    this->model_.lock()->GetFrameTransform(parent_frame_index_, other);
  return transform_function_(parent_transform).inverse() * parent_transform;
}

}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::LogicalFrame)
