#pragma once

#include "common/Types.h"

namespace ocs2 {
namespace legged_robot {
namespace arm {

struct GripperStatesForPlan
{
    vector3_t  x;
    vector3_t  y;
    vector3_t  z;
    vector3_t  yaw;
    vector3_t  pitch;
    vector3_t  roll;
};

struct BaseStatesForPlan
{
    vector3_t  x;
    vector3_t  y;
    vector3_t  z;
    vector3_t  yaw;
    vector3_t  pitch;
    vector3_t  roll;
};

struct GripperBasePosition {
    scalar_t base_x;
    scalar_t base_y;
    scalar_t base_z;
    scalar_t base_yaw;
    scalar_t base_pitch;
    scalar_t base_roll;

    scalar_t gripper_x;
    scalar_t gripper_y;
    scalar_t gripper_z;
    scalar_t gripper_yaw;
    scalar_t gripper_pitch;
    scalar_t gripper_roll;
};

struct GripperBaseVelocity {
    scalar_t base_x;
    scalar_t base_y;
    scalar_t base_z;
    scalar_t base_yaw;
    scalar_t base_pitch;
    scalar_t base_roll;

    scalar_t gripper_x;
    scalar_t gripper_y;
    scalar_t gripper_z;
    scalar_t gripper_yaw;
    scalar_t gripper_pitch;
    scalar_t gripper_roll;
};

} // namespace arm
} 
} // namespace ocs2

