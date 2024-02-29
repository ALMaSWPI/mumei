#include "huron/multibody/model.h"
#include "huron/multibody/model_impl_factory.h"

namespace huron {
namespace multibody {

template <typename T>
Model<T>::Model() = default;

template <typename T>
void Model<T>::AddModelImpl(ModelImplType type,
                         bool set_as_default) {
  impls_.push_back(internal::ModelImplFactory<T>::Create(type));
  if (set_as_default) {
    default_impl_index_ = impls_.size() - 1;
  }
}

template <typename T>
internal::ModelImplInterface<T> const * Model<T>::GetModelImpl(size_t index) const {
  return impls_[index].get();
}

template <typename T>
void Model<T>::BuildFromUrdf(const std::string& urdf_path,
                          JointType root_joint_type) {
  assert(!is_finalized_);
  if (impls_.empty()) {
    throw std::runtime_error("No model implementations.");
  }
  for (auto& impl : impls_) {
    impl->BuildFromUrdf(urdf_path, root_joint_type);
  }
  num_positions_ = impls_[default_impl_index_]->num_positions();
  num_velocities_ = impls_[default_impl_index_]->num_velocities();
  // Initialize the joint vector
  joints_ = std::vector<std::shared_ptr<Joint<T>>>(
      impls_[default_impl_index_]->num_joints());
  // Initialize the frame vector
  frames_ = std::vector<std::shared_ptr<Frame<T>>>(
      impls_[default_impl_index_]->num_frames());

  auto joint_names = impls_[default_impl_index_]->GetJointNames();
  size_t last_position_index = 0, last_velocity_index = 0;
  for (auto i = 0; i < joints_.size(); ++i) {
    auto jd_tmp_ptr =
      impls_[default_impl_index_]->GetJointDescription(joint_names[i]);
    // Add joint
    AddJoint(i, std::move(jd_tmp_ptr));
    // Set id_q and id_v
    joints_[i]->SetIndices(last_position_index, last_velocity_index);
    last_position_index += joints_[i]->Info()->num_positions();
    last_velocity_index += joints_[i]->Info()->num_velocities();
  }
  for (auto i = 0; i < frames_.size(); ++i) {
    // Add frame
    DoAddFrameFromModelDescription(
      i,
      impls_[default_impl_index_]->GetFrameName(i),
      impls_[default_impl_index_]->GetFrameType(i));
  }
  is_constructed_ = true;
}

template <typename T>
void Model<T>::Finalize(const huron::VectorX<T>& initial_state) {
  assert(is_constructed_);
  // Check if all joints are added to the model, and set the joint indices.
  for (auto& joint : joints_) {
    assert(joint != nullptr);
    assert(joint->IsFullyConfigured());
  }
  // // Resize the state vector
  // states_.resize(num_positions_ + num_velocities_);
  // Set the initial state
  assert((initial_state.size() == num_positions_ + num_velocities_) &&
         "The initial state dimension does not match the model.");
  states_ = initial_state;
  is_finalized_ = true;
}

template <typename T>
void Model<T>::Finalize() {
  assert(is_constructed_);
  Finalize(huron::VectorX<T>::Zero(num_positions_ + num_velocities_));
}

template <typename T>
Joint<T>* const Model<T>::GetJoint(JointIndex index) {
  assert(is_constructed_);
  return joints_[index].get();
}

template <typename T>
Joint<T>* const Model<T>::GetJoint(const std::string& name) {
  assert(is_constructed_);
  return joints_[GetJointIndex(name)].get();
}

template <typename T>
void Model<T>::SetJointStateProvider(
  JointIndex index,
  std::shared_ptr<StateProvider<T>> state_provider) {
  assert(is_constructed_);
  joints_[index]->SetStateProvider(std::move(state_provider));
}

template <typename T>
JointIndex Model<T>::GetJointIndex(const std::string& joint_name) const {
  return impls_[default_impl_index_]->GetJointIndex(joint_name);
}

template <typename T>
void Model<T>::DoAddFrameFromModelDescription(FrameIndex idx,
                                           const std::string& name,
                                           FrameType type) {
  frames_[idx] = Frame<T>::make_shared(
    idx,
    name,
    type,
    false,
    this->weak_from_this());
  frame_name_to_index_[name] = idx;
}

template <typename T>
void Model<T>::UpdateJointStates() {
  assert(is_finalized_);
  for (auto& joint : joints_) {
    if (joint->Info()->type() == JointType::kUnknown) {
      continue;
    }
    joint->UpdateState();
    states_.segment(joint->id_q(),
                    joint->Info()->num_positions()) = joint->GetPositions();
    states_.segment(num_positions_ + joint->id_v(),
                    joint->Info()->num_velocities()) = joint->GetVelocities();
  }
}

template <typename T>
std::weak_ptr<const Frame<T>> Model<T>::GetFrame(FrameIndex index) const {
  return frames_[index];
}

template <typename T>
std::weak_ptr<const Frame<T>> Model<T>::GetFrame(const std::string& name) const {
  return frames_[frame_name_to_index_.at(name)];
}

// Kinematics and Dynamics functions
template <typename T>
huron::SE3<T> Model<T>::GetJointTransformInWorld(size_t joint_index) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetJointTransformInWorld(joint_index);
}

template <typename T>
FrameIndex Model<T>::GetFrameIndex(const std::string& frame_name) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetFrameIndex(frame_name);
}

template <typename T>
huron::SE3<T> Model<T>::GetFrameTransform(FrameIndex from_frame,
                                         FrameIndex to_frame) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetFrameTransform(from_frame, to_frame);
}

template <typename T>
huron::SE3<T> Model<T>::GetFrameTransformInWorld(FrameIndex frame) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetFrameTransformInWorld(frame);
}

template <typename T>
huron::VectorX<T> Model<T>::NeutralConfiguration() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->NeutralConfiguration();
}

template <typename T>
huron::Vector3<T> Model<T>::EvalCenterOfMassPosition() {
  assert(is_finalized_);
  return impls_[default_impl_index_]->EvalCenterOfMassPosition();
}

template <typename T>
huron::Vector3<T> Model<T>::GetCenterOfMassPosition() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCenterOfMassPosition();
}

template <typename T>
const Eigen::VectorBlock<const huron::VectorX<T>> Model<T>::GetPositions() const {
  assert(is_finalized_);
  return states_.segment(0, num_positions_);
}

template <typename T>
const Eigen::VectorBlock<const huron::VectorX<T>> Model<T>::GetVelocities() const {
  assert(is_finalized_);
  return states_.segment(num_positions_, num_velocities_);
}

template <typename T>
const huron::VectorX<T>& Model<T>::GetAccelerations() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetAccelerations();
}

template <typename T>
const huron::VectorX<T>& Model<T>::GetTorques() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetTorques();
}

template <typename T>
const huron::MatrixX<T>& Model<T>::GetMassMatrix() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetMassMatrix();
}

template <typename T>
const huron::MatrixX<T>& Model<T>::GetCoriolisMatrix() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCoriolisMatrix();
}

template <typename T>
const huron::VectorX<T>& Model<T>::GetNonlinearEffects() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetNonlinearEffects();
}

template <typename T>
const huron::VectorX<T>& Model<T>::GetGravity() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetGravity();
}

template <typename T>
const huron::Vector6<T>& Model<T>::GetSpatialMomentum() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetSpatialMomentum();
}

template <typename T>
huron::Vector6<T> Model<T>::GetCentroidalMomentum() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCentroidalMomentum();
}

template <typename T>
const huron::Matrix6X<T>& Model<T>::GetCentroidalMatrix() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCentroidalMatrix();
}

template <typename T>
void Model<T>::ComputeAll() {
  assert(is_finalized_);
  impls_[default_impl_index_]->ComputeAll(
    states_.segment(0, num_positions_),
    states_.segment(num_positions_, num_velocities_));
}

template <typename T>
void Model<T>::ForwardKinematics() {
  assert(is_finalized_);
  impls_[default_impl_index_]->ForwardKinematics(
    states_.segment(0, num_positions_),
    states_.segment(num_positions_, num_velocities_));
}

}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::Model)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::Model)
