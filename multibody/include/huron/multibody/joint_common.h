#pragma once

#include <eigen3/Eigen/Dense>

#include <stddef.h>
#include <string>
#include <cassert>
#include <limits>

#include "huron/types.h"
#include "huron/utils/template_instantiations.h"
#include "frame.h"

namespace huron {
namespace multibody {

using JointIndex = size_t;

enum class JointType {
  kUnknown = 0,
  kFixed,
  kRevolute,
  kPrismatic,
  kPlanar,
  kSpherical,
  kFreeFlyer
};

template <typename T>
struct JointDescription {
 public:
  // TODO(dtbpkmte): Properly implement JointIndex/FrameIndex and casts.
  JointDescription(size_t id, const std::string& name,
                   size_t parent_frame_id,
                   size_t child_frame_id,
                   size_t num_positions, size_t num_velocities,
                   JointType type,
                   const huron::VectorX<T>& min_position,
                   const huron::VectorX<T>& max_position,
                   const huron::VectorX<T>& min_velocity,
                   const huron::VectorX<T>& max_velocity,
                   const huron::VectorX<T>& min_acceleration,
                   const huron::VectorX<T>& max_acceleration,
                   const huron::VectorX<T>& min_torque,
                   const huron::VectorX<T>& max_torque,
                   const huron::VectorX<T>& friction,
                   const huron::VectorX<T>& damping)
      : id_((JointIndex) id),
        name_(name),
        parent_frame_id_((FrameIndex) parent_frame_id),
        child_frame_id_((FrameIndex) child_frame_id),
        num_positions_(num_positions),
        num_velocities_(num_velocities),
        type_(type),
        min_position_(min_position),
        max_position_(max_position),
        min_velocity_(min_velocity),
        max_velocity_(max_velocity),
        min_acceleration_(min_acceleration),
        max_acceleration_(max_acceleration),
        min_torque_(min_torque),
        max_torque_(max_torque),
        friction_(friction),
        damping_(damping) {
    // assert(min_position.size() == num_positions_);
    // assert(max_position.size() == num_positions_);
    // assert((max_position.array() >= min_position.array()).all());
    //
    // assert(min_velocity.size() == num_velocities_);
    // assert(max_velocity.size() == num_velocities_);
    // assert((max_velocity.array() >= min_velocity.array()).all());
    //
    // assert(min_acceleration.size() == num_velocities_);
    // assert(max_acceleration.size() == num_velocities_);
    // assert((max_acceleration.array() >= min_acceleration.array()).all());
    //
    // assert(min_torque.size() == num_velocities_);
    // assert(max_torque.size() == num_velocities_);
    // assert((max_torque.array() >= min_torque.array()).all());
    //
    // assert(friction.size() == num_velocities_);
    // assert(damping.size() == num_velocities_);
  }

  JointDescription(size_t id, const std::string& name,
                   size_t parent_frame_id,
                   size_t child_frame_id,
                   size_t num_positions, size_t num_velocities,
                   JointType type,
                   const huron::VectorX<T>& min_position,
                   const huron::VectorX<T>& max_position,
                   const huron::VectorX<T>& min_velocity,
                   const huron::VectorX<T>& max_velocity,
                   const huron::VectorX<T>& min_acceleration,
                   const huron::VectorX<T>& max_acceleration,
                   const huron::VectorX<T>& min_torque,
                   const huron::VectorX<T>& max_torque)
      : JointDescription(id, name,
                         parent_frame_id, child_frame_id,
                         num_positions, num_velocities,
                         type,
                         min_position, max_position,
                         min_velocity, max_velocity,
                         min_acceleration, max_acceleration,
                         min_torque, max_torque,
                         huron::VectorX<T>::Zero(num_velocities),
                         huron::VectorX<T>::Zero(num_velocities)) {}

  JointDescription(size_t id, const std::string& name,
                   size_t parent_frame_id,
                   size_t child_frame_id,
                   size_t num_positions,
                   size_t num_velocities,
                   JointType type)
      : JointDescription(
          id, name,
          parent_frame_id, child_frame_id,
          num_positions, num_velocities,
          type,
          huron::VectorX<T>::Constant(num_positions,
            -std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_positions,
            std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_velocities,
            -std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_velocities,
            std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_velocities,
            -std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_velocities,
            std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_velocities,
            -std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Constant(num_velocities,
            std::numeric_limits<double>::infinity()),
          huron::VectorX<T>::Zero(num_velocities),
          huron::VectorX<T>::Zero(num_velocities)) {}

  JointIndex id() const { return id_; }
  const std::string& name() const { return name_; }
  FrameIndex parent_frame_id() const { return parent_frame_id_; }
  FrameIndex child_frame_id() const { return child_frame_id_; }
  size_t num_positions() const { return num_positions_; }
  size_t num_velocities() const { return num_velocities_; }
  JointType type() const { return type_; }
  const huron::VectorX<T>& min_position() const { return min_position_; }
  const huron::VectorX<T>& max_position() const { return max_position_; }
  const huron::VectorX<T>& min_velocity() const { return min_velocity_; }
  const huron::VectorX<T>& max_velocity() const { return max_velocity_; }
  const huron::VectorX<T>& min_acceleration() const { return min_acceleration_; }
  const huron::VectorX<T>& max_acceleration() const { return max_acceleration_; }
  const huron::VectorX<T>& min_torque() const { return min_torque_; }
  const huron::VectorX<T>& max_torque() const { return max_torque_; }
  const huron::VectorX<T>& friction() const { return friction_; }
  const huron::VectorX<T>& damping() const { return damping_; }

 private:
  JointIndex id_;
  std::string name_;
  FrameIndex parent_frame_id_;
  FrameIndex child_frame_id_;
  size_t num_positions_;
  size_t num_velocities_;
  JointType type_;
  huron::VectorX<T> min_position_;
  huron::VectorX<T> max_position_;
  huron::VectorX<T> min_velocity_;
  huron::VectorX<T> max_velocity_;
  huron::VectorX<T> min_acceleration_;
  huron::VectorX<T> max_acceleration_;
  huron::VectorX<T> min_torque_;
  huron::VectorX<T> max_torque_;
  huron::VectorX<T> friction_;
  huron::VectorX<T> damping_;
};

template <typename T>
std::ostream& operator<<(std::ostream &os, const JointDescription<T> &jd) {
  return (os << "ID: " << jd.id()
             << "\nName: " << jd.name()
             << "\nParent Frame ID: " << jd.parent_frame_id()
             << "\nChild Frame ID: " << jd.child_frame_id()
             << "\nNum Positions: " << jd.num_positions()
             << "\nNum Velocities: " << jd.num_velocities()
             << "\nType: " << static_cast<size_t>(jd.type())
             << "\nMin Position: " << jd.min_position()
             << "\nMax Position: " << jd.max_position()
             << "\nMin Velocity: " << jd.min_velocity()
             << "\nMax Velocity: " << jd.max_velocity()
             << "\nMin Acceleration: " << jd.min_acceleration()
             << "\nMax Acceleration: " << jd.max_acceleration()
             << "\nMin Torque: " << jd.min_torque()
             << "\nMax Torque: " << jd.max_torque()
             << "\nFriction: " << jd.friction()
             << "\nDamping: " << jd.damping()
             << std::endl);
}

}  // namespace multibody
}  // namespace huron
