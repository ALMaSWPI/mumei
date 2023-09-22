#pragma once

namespace huron {

  class GenericComponent {
   public:
    virtual void Configure() = 0;
    virtual void Initialize() = 0;
    virtual void SetUp() = 0;
    virtual void Terminate() = 0;
  };

}// namespace huron
