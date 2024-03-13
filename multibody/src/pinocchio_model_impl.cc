#include "pinocchio/math/casadi.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include "huron/multibody/pinocchio_model_impl.h"
#include "huron/multibody/joint_common.h"
#include "huron/multibody/model_impl_types.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

namespace helpers {

template <typename T>
huron::SE3<T> PinSe3ToHuronSe3(const pinocchio::SE3Tpl<T>& se3) {
  huron::SE3<T> ret(se3.toHomogeneousMatrix());
  return ret;
}

}  // namespace helpers

template <typename T>
struct PinocchioModelImpl<T>::Impl {
  mutable pinocchio::ModelTpl<T> model_;
  mutable pinocchio::DataTpl<T> data_;
};

template <typename T>
PinocchioModelImpl<T>::PinocchioModelImpl()
  : impl_(std::make_unique<Impl>()) {}

template <typename T>
PinocchioModelImpl<T>::~PinocchioModelImpl() = default;

template <>
void PinocchioModelImpl<double>::BuildFromUrdf(const std::string& urdf_path,
                                       JointType root_joint_type) {
  pinocchio::Model::JointModel joint_model;
  if (root_joint_type == JointType::kFixed) {
    pinocchio::urdf::buildModel(urdf_path, impl_->model_);
  } else {
    switch (root_joint_type) {
      case JointType::kFreeFlyer:
        joint_model = pinocchio::JointModelFreeFlyer();
        break;
      case JointType::kPlanar:
        joint_model = pinocchio::JointModelPlanar();
        break;
      default:
        throw std::runtime_error("Unsupported root joint type.");
        break;
    }
    pinocchio::urdf::buildModel(urdf_path, joint_model, impl_->model_);
  }
  impl_->data_ = pinocchio::Data(impl_->model_);
  num_positions_ = impl_->model_.nq;
  num_velocities_ = impl_->model_.nv;
  num_joints_ = impl_->model_.njoints;
  num_frames_ = impl_->model_.nframes;
}

template <typename T>
void PinocchioModelImpl<T>::BuildFromUrdf(const std::string& urdf_path,
                                       JointType root_joint_type) {
  pinocchio::Model tmp_model;
  pinocchio::Model::JointModel joint_model;
  if (root_joint_type == JointType::kFixed) {
    pinocchio::urdf::buildModel(urdf_path, tmp_model);
    impl_->model_ = tmp_model.cast<T>();
  } else {
    switch (root_joint_type) {
      case JointType::kFreeFlyer:
        joint_model = pinocchio::JointModelFreeFlyer();
        break;
      case JointType::kPlanar:
        joint_model = pinocchio::JointModelPlanar();
        break;
      default:
        throw std::runtime_error("Unsupported root joint type.");
        break;
    }
    pinocchio::urdf::buildModel(urdf_path, joint_model, tmp_model);
    impl_->model_ = tmp_model.cast<T>();
  }
  impl_->data_ = pinocchio::DataTpl<T>(impl_->model_);
  num_positions_ = impl_->model_.nq;
  num_velocities_ = impl_->model_.nv;
  num_joints_ = impl_->model_.njoints;
  num_frames_ = impl_->model_.nframes;
}

template <typename T>
const std::vector<std::string>& PinocchioModelImpl<T>::GetJointNames() const {
  return impl_->model_.names;
}

template <typename T>
std::weak_ptr<Joint<T>>
PinocchioModelImpl<T>::GetJoint(const std::string& name) const {
  throw NotImplementedException();
}

template <typename T>
std::weak_ptr<Joint<T>>
PinocchioModelImpl<T>::GetJoint(size_t joint_index) const {
  throw NotImplementedException();
}

template <typename T>
std::unique_ptr<JointDescription<T>> PinocchioModelImpl<T>::GetJointDescription(
  JointIndex joint_index) const {
  return GetJointDescription(impl_->model_.names[joint_index]);
}

template <typename T>
std::unique_ptr<JointDescription<T>>
PinocchioModelImpl<T>::GetJointDescription(
  const std::string& joint_name) const {
  // auto frame_id = impl_->model_.getFrameId(joint_name, pinocchio::JOINT);
  auto frame_id = impl_->model_.getFrameId(joint_name);
  assert(frame_id < impl_->model_.nframes);
  auto frame = impl_->model_.frames[frame_id];
  size_t joint_index = frame.parent;
  size_t parent_frame_index =  frame.previousFrame;
  JointType joint_type = (frame_id == 0) ? JointType::kUnknown
                                         : GetJointType(joint_index);
  return std::make_unique<JointDescription<T>>(
    frame.parent,
    joint_name,
    parent_frame_index,
    frame_id + 1,  // child frame seems to be the immediate next frame
    impl_->model_.nqs[joint_index],
    impl_->model_.nvs[joint_index],
    joint_type,
    impl_->model_.lowerPositionLimit.segment(impl_->model_.idx_qs[joint_index],
                                      impl_->model_.nqs[joint_index]),
    impl_->model_.upperPositionLimit.segment(impl_->model_.idx_qs[joint_index],
                                      impl_->model_.nqs[joint_index]),
    -impl_->model_.velocityLimit.segment(impl_->model_.idx_vs[joint_index],
                                 impl_->model_.nvs[joint_index]),
    impl_->model_.velocityLimit.segment(impl_->model_.idx_vs[joint_index],
                                 impl_->model_.nvs[joint_index]),
    huron::VectorX<T>::Constant(impl_->model_.nvs[joint_index],
                              -std::numeric_limits<double>::infinity()),
    huron::VectorX<T>::Constant(impl_->model_.nvs[joint_index],
                              std::numeric_limits<double>::infinity()),
    -impl_->model_.effortLimit.segment(impl_->model_.idx_vs[joint_index],
                               impl_->model_.nvs[joint_index]),
    impl_->model_.effortLimit.segment(impl_->model_.idx_vs[joint_index],
                               impl_->model_.nvs[joint_index]),
    impl_->model_.friction.segment(impl_->model_.idx_vs[joint_index],
                            impl_->model_.nvs[joint_index]),
    impl_->model_.damping.segment(impl_->model_.idx_vs[joint_index],
                           impl_->model_.nvs[joint_index]));
}

template <typename T>
huron::SE3<T>
PinocchioModelImpl<T>::GetJointTransformInWorld(size_t joint_index) const {
  return helpers::PinSe3ToHuronSe3(impl_->data_.oMi[joint_index]);
}

template <typename T>
JointIndex
PinocchioModelImpl<T>::GetJointIndex(const std::string& joint_name) const {
  return impl_->model_.getJointId(joint_name);
}

template <typename T>
FrameIndex PinocchioModelImpl<T>::GetFrameIndex(
  const std::string& frame_name) const {
  return impl_->model_.getFrameId(frame_name);
}

template <typename T>
const std::string&
PinocchioModelImpl<T>::GetFrameName(FrameIndex frame_index) const {
  return impl_->model_.frames[frame_index].name;
}

template <typename T>
FrameType PinocchioModelImpl<T>::GetFrameType(FrameIndex frame_index) const {
  if (impl_->model_.frames[frame_index].type == pinocchio::BODY) {
    return FrameType::kPhysical;
  } else if (impl_->model_.frames[frame_index].type == pinocchio::JOINT) {
    return FrameType::kJoint;
  } else if (impl_->model_.frames[frame_index].type == pinocchio::SENSOR) {
    return FrameType::kSensor;
  } else if (impl_->model_.frames[frame_index].type == pinocchio::FIXED_JOINT) {
    return FrameType::kFixed;
  } else {
    throw std::runtime_error("Unknown frame type.");
  }
}

template <typename T>
huron::SE3<T>
PinocchioModelImpl<T>::GetFrameTransform(FrameIndex from_frame,
                                      FrameIndex to_frame) const {
  return GetFrameTransformInWorld(from_frame).Inverse() *
         GetFrameTransformInWorld(to_frame);
}

template <typename T>
huron::SE3<T>
PinocchioModelImpl<T>::GetFrameTransformInWorld(FrameIndex frame) const {
  pinocchio::updateFramePlacement(
      impl_->model_,
      impl_->data_,
      static_cast<size_t>(frame));
  return helpers::PinSe3ToHuronSe3(impl_->data_.oMf[frame]);
}

template <typename T>
huron::Vector3<T> PinocchioModelImpl<T>::EvalCenterOfMassPosition() {
  return pinocchio::centerOfMass(impl_->model_, impl_->data_);
}

template <typename T>
huron::Vector3<T> PinocchioModelImpl<T>::GetCenterOfMassPosition() const {
  return impl_->data_.com[0];
}

template <typename T>
huron::VectorX<T> PinocchioModelImpl<T>::NeutralConfiguration() const {
  return pinocchio::neutral(impl_->model_);
}

template <typename T>
const huron::VectorX<T>& PinocchioModelImpl<T>::GetAccelerations() const {
  return impl_->data_.ddq;
}
template <typename T>
const huron::VectorX<T>& PinocchioModelImpl<T>::GetTorques() const {
  return impl_->data_.tau;
}
template <typename T>
const huron::MatrixX<T>& PinocchioModelImpl<T>::GetMassMatrix() const {
  return impl_->data_.M;
}
template <typename T>
const huron::MatrixX<T>& PinocchioModelImpl<T>::GetCoriolisMatrix() const {
  return impl_->data_.C;
}
template <typename T>
const huron::VectorX<T>& PinocchioModelImpl<T>::GetNonlinearEffects() const {
  return impl_->data_.nle;
}
template <typename T>
const huron::VectorX<T>& PinocchioModelImpl<T>::GetGravity() const {
  return impl_->data_.g;
}
template <typename T>
const huron::Vector6<T>& PinocchioModelImpl<T>::GetSpatialMomentum() const {
  throw NotImplementedException();
}
template <typename T>
huron::Vector6<T> PinocchioModelImpl<T>::GetCentroidalMomentum() const {
  return impl_->data_.hg;
}
template <typename T>
const huron::Matrix6X<T>& PinocchioModelImpl<T>::GetCentroidalMatrix() const {
  return impl_->data_.Ag;
}

template <typename T>
void PinocchioModelImpl<T>::ComputeAll(
  const Eigen::Ref<const huron::VectorX<T>>& q,
  const Eigen::Ref<const huron::VectorX<T>>& v) {
  pinocchio::computeAllTerms(impl_->model_, impl_->data_, q, v);
}

template <typename T>
void PinocchioModelImpl<T>::ForwardKinematics(
  const Eigen::Ref<const huron::VectorX<T>>& q) {
  pinocchio::forwardKinematics(impl_->model_, impl_->data_, q);
}
template <typename T>
void PinocchioModelImpl<T>::ForwardKinematics(
  const Eigen::Ref<const huron::VectorX<T>>& q,
  const Eigen::Ref<const huron::VectorX<T>>& v) {
  pinocchio::forwardKinematics(impl_->model_, impl_->data_, q, v);
}
template <typename T>
void PinocchioModelImpl<T>::ForwardKinematics(
  const Eigen::Ref<const huron::VectorX<T>>& q,
  const Eigen::Ref<const huron::VectorX<T>>& v,
  const Eigen::Ref<const huron::VectorX<T>>& a) {
  pinocchio::forwardKinematics(impl_->model_, impl_->data_, q, v, a);
}

template <typename T>
JointType PinocchioModelImpl<T>::GetJointType(size_t joint_index) const {
  if (impl_->model_.joints[joint_index].shortname() == "JointModelFreeFlyer") {
    return JointType::kFreeFlyer;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelRX") {
    return JointType::kRevolute;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelRY") {
    return JointType::kRevolute;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelRZ") {
    return JointType::kRevolute;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPX") {
    return JointType::kPrismatic;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPY") {
    return JointType::kPrismatic;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPZ") {
    return JointType::kPrismatic;
  } else if (impl_->model_.joints[joint_index].shortname() ==
      "JointModelSpherical") {
    return JointType::kSpherical;
  } else if (impl_->model_.joints[joint_index].shortname() ==
      "JointModelPlanar") {
    return JointType::kPlanar;
  } else {
    return JointType::kUnknown;
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::internal::PinocchioModelImpl)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::internal::PinocchioModelImpl)
