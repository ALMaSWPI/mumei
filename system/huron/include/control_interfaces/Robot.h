#include "control_interfaces/Limb.h"

class Robot {
public:
    void Init(Limb limb_list[]);
private:
    Limb limbs_;
};

