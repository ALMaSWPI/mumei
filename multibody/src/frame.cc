#include "huron/multibody/frame.h"
#include "huron/multibody/model.h"

namespace huron {
namespace multibody {

template <typename T>
Frame<T>::Frame(FrameIndex index,
             const std::string& name,
             FrameType type,
             bool is_user_defined,
             std::weak_ptr<const Model<T>> model)
  : index_(index),
    name_(name),
    type_(type),
    is_user_defined_(is_user_defined),
    model_(std::move(model)) {}

template <typename T>
huron::SE3<T> Frame<T>::GetTransformInWorld() const {
  return model_.lock()->GetFrameTransformInWorld(index_);
}

template <typename T>
huron::SE3<T> Frame<T>::GetTransformFromFrame(const Frame& other) const {
  return model_.lock()->GetFrameTransform(other.index_, index_);
}

template <typename T>
huron::SE3<T> Frame<T>::GetTransformFromFrame(FrameIndex other) const {
  return model_.lock()->GetFrameTransform(other, index_);
}

template <typename T>
huron::SE3<T> Frame<T>::GetTransformToFrame(const Frame& other) const {
  return model_.lock()->GetFrameTransform(index_, other.index_);
}

template <typename T>
huron::SE3<T> Frame<T>::GetTransformToFrame(FrameIndex other) const {
  return model_.lock()->GetFrameTransform(index_, other);
}

}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::Frame)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::Frame)
