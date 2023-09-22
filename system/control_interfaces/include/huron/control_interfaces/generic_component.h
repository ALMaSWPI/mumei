#pragma once

namespace huron {

/**
 * Interface for all components.
 *
 * @ingroup control_interfaces
 */
class GenericComponent {
 public:
  virtual void Configure() = 0;
  virtual void Initialize() = 0;
  virtual void SetUp() = 0;
  virtual void Terminate() = 0;
};

}// namespace huron
