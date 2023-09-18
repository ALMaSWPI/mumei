#pragma once

#define AXIS_COUNT (2)

// ODrive.Controller.ControlMode
enum ControlMode {
    CONTROL_MODE_VOLTAGE_CONTROL             = 0,
    CONTROL_MODE_TORQUE_CONTROL              = 1,
    CONTROL_MODE_VELOCITY_CONTROL            = 2,
    CONTROL_MODE_POSITION_CONTROL            = 3,
};

// ODrive.Controller.InputMode
enum InputMode {
    INPUT_MODE_INACTIVE                      = 0,
    INPUT_MODE_PASSTHROUGH                   = 1,
    INPUT_MODE_VEL_RAMP                      = 2,
    INPUT_MODE_POS_FILTER                    = 3,
    INPUT_MODE_MIX_CHANNELS                  = 4,
    INPUT_MODE_TRAP_TRAJ                     = 5,
    INPUT_MODE_TORQUE_RAMP                   = 6,
    INPUT_MODE_MIRROR                        = 7,
    INPUT_MODE_TUNING                        = 8,
};
