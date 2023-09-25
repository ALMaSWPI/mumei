#pragma once

#include <vector>
#include "generic_component.h"

namespace huron {

/**
 * Interface for components that can move.
 *
 * @ingroup control_interfaces
 */
class MovingComponent : public GenericComponent {
 public:

  /**
   * Moves the component by the specified input.
   *
   * @param value Input value.
   * @return true if the operation is successful, false otherwise.
   */
  virtual bool Move(float value) = 0;

  /**
   * Moves the component by the specified input vector.
   *
   * This method can be used if the component needs more than one input.
   * For example, a position controlled motor needs position input,
   * velocity feedforward, and current feedforward.
   *
   * @param value Input value vector.
   * @return true if the operation is successful, false otherwise.
   */
  virtual bool Move(const std::vector<float>& values) = 0;

  /**
   * Stops the component from moving.
   *
   * @return true if the operation is successful, false otherwise.
   */
  virtual bool Stop() = 0;
};

}  // namespace huron
