#include "control_interfaces/Joint.h"

class Limb {
public:
    void Init(Joint joint_list[]);
    void AddJoint(Joint joint);
    bool Move(float values[]);
    bool Stop();

private:
    std::vector<Joint> joints_[];
};


#endif