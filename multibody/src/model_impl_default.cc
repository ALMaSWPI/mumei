#include "huron/multibody/model_impl_interface.h"

namespace huron {
namespace multibody {
namespace internal {

template <typename T>
void ModelImplInterface<T>::BuildFromUrdf(const std::string& urdf_path,
                                       JointType root_joint_type) {
  throw NotImplementedException();
}

template <typename T>
const std::vector<std::string>& ModelImplInterface<T>::GetJointNames() const {
  throw NotImplementedException();
}

template <typename T>
std::weak_ptr<Joint<T>>
ModelImplInterface<T>::GetJoint(const std::string& name) const {
  throw NotImplementedException();
}

template <typename T>
std::weak_ptr<Joint<T>>
ModelImplInterface<T>::GetJoint(size_t joint_index) const {
  throw NotImplementedException();
}

template <typename T>
JointType ModelImplInterface<T>::GetJointType(size_t joint_index) const {
  throw NotImplementedException();
}

template <typename T>
JointIndex
ModelImplInterface<T>::GetJointIndex(const std::string& joint_name) const {
  throw NotImplementedException();
}

template <typename T>
std::unique_ptr<JointDescription<T>> ModelImplInterface<T>::GetJointDescription(
  JointIndex joint_index) const {
  throw NotImplementedException();
}
template <typename T>
std::unique_ptr<JointDescription<T>> ModelImplInterface<T>::GetJointDescription(
  const std::string& joint_name) const {
  throw NotImplementedException();
}

template <typename T>
huron::SE3<T>
ModelImplInterface<T>::GetJointTransformInWorld(size_t joint_index) const {
  throw NotImplementedException();
}

template <typename T>
FrameIndex ModelImplInterface<T>::GetFrameIndex(
  const std::string& frame_name) const {
  throw NotImplementedException();
}

template <typename T>
const std::string&
ModelImplInterface<T>::GetFrameName(FrameIndex frame_index) const {
  throw NotImplementedException();
}

template <typename T>
FrameType ModelImplInterface<T>::GetFrameType(FrameIndex frame_index) const {
  throw NotImplementedException();
}

template <typename T>
huron::SE3<T>
ModelImplInterface<T>::GetFrameTransform(FrameIndex from_frame,
                                      FrameIndex to_frame) const {
  throw NotImplementedException();
}

template <typename T>
huron::SE3<T>
ModelImplInterface<T>::GetFrameTransformInWorld(FrameIndex frame) const {
  throw NotImplementedException();
}

template <typename T>
huron::Vector3<T> ModelImplInterface<T>::EvalCenterOfMassPosition() {
  throw NotImplementedException();
}

template <typename T>
huron::Vector3<T> ModelImplInterface<T>::GetCenterOfMassPosition() const {
  throw NotImplementedException();
}

template <typename T>
huron::VectorX<T> ModelImplInterface<T>::NeutralConfiguration() const {
  throw NotImplementedException();
}

template <typename T>
const huron::VectorX<T>& ModelImplInterface<T>::GetAccelerations() const {
  throw NotImplementedException();
}

template <typename T>
const huron::VectorX<T>& ModelImplInterface<T>::GetTorques() const {
  throw NotImplementedException();
}

template <typename T>
const huron::MatrixX<T>& ModelImplInterface<T>::GetMassMatrix() const {
  throw NotImplementedException();
}

template <typename T>
const huron::MatrixX<T>& ModelImplInterface<T>::GetCoriolisMatrix() const {
  throw NotImplementedException();
}

template <typename T>
const huron::VectorX<T>& ModelImplInterface<T>::GetNonlinearEffects() const {
  throw NotImplementedException();
}

template <typename T>
const huron::VectorX<T>& ModelImplInterface<T>::GetGravity() const {
  throw NotImplementedException();
}

template <typename T>
const huron::Vector6<T>& ModelImplInterface<T>::GetSpatialMomentum() const {
  throw NotImplementedException();
}

template <typename T>
huron::Vector6<T> ModelImplInterface<T>::GetCentroidalMomentum() const {
  throw NotImplementedException();
}

template <typename T>
const huron::Matrix6X<T>& ModelImplInterface<T>::GetCentroidalMatrix() const {
  throw NotImplementedException();
}

template <typename T>
void ModelImplInterface<T>::ComputeAll(
  const Eigen::Ref<const huron::VectorX<T>>& q,
  const Eigen::Ref<const huron::VectorX<T>>& v) {
  throw NotImplementedException();
}

template <typename T>
void ModelImplInterface<T>::ForwardKinematics(
  const Eigen::Ref<const huron::VectorX<T>>& q) {
  throw NotImplementedException();
}
template <typename T>
void ModelImplInterface<T>::ForwardKinematics(
  const Eigen::Ref<const huron::VectorX<T>>& q,
  const Eigen::Ref<const huron::VectorX<T>>& v) {
  throw NotImplementedException();
}
template <typename T>
void ModelImplInterface<T>::ForwardKinematics(
  const Eigen::Ref<const huron::VectorX<T>>& q,
  const Eigen::Ref<const huron::VectorX<T>>& v,
  const Eigen::Ref<const huron::VectorX<T>>& a) {
  throw NotImplementedException();
}

template <typename T>
bool ModelImplInterface<T>::is_built() const {
  throw NotImplementedException();
}

template <typename T>
size_t ModelImplInterface<T>::num_positions() const {
  throw NotImplementedException();
}
template <typename T>
size_t ModelImplInterface<T>::num_velocities() const {
  throw NotImplementedException();
}
template <typename T>
size_t ModelImplInterface<T>::num_joints() const {
  throw NotImplementedException();
}
template <typename T>
size_t ModelImplInterface<T>::num_frames() const {
  throw NotImplementedException();
}

}  // namespace internal
}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::internal::ModelImplInterface)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::internal::ModelImplInterface)
