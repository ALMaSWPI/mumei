#pragma once

#include "limb.h"

namespace huron {

  class Robot {
   public:
    void Init(std::vector<Limb> limbs);

   private:
    std::vector<Limb> limbs_;
  };

}// namespace huron
