#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "common/Types.h"

namespace ocs2 {
namespace legged_robot {
namespace arm {

struct ArmSettings {
    // This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
    std::vector<std::string> jointNames{"ARM_J1", "ARM_J2", "ARM_J3", "ARM_J4", "ARM_J5", "ARM_J6"};
    std::vector<std::string> endEffectorNames6DoF{"Gripper_Point", "base"};
};

} // namespace arm
} // namespace legged_robot
} // namespace ocs2