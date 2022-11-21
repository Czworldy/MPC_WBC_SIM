#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

struct ModelSettings {
    scalar_t positionErrorGain = 0.0;

    scalar_t phaseTransitionStanceTime = 0.4;

    bool verboseCppAd = true;
    bool recompileLibrariesCppAd = true;
    std::string modelFolderCppAd = "/tmp/ocs2";

    // This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
    std::vector<std::string> jointNames{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", 
                                        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
                                        "ARM_J1", "ARM_J2", "ARM_J3", "ARM_J4", "ARM_J5", "ARM_J6"};
    std::vector<std::string> contactNames6DoF{};
    std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT","LH_FOOT", "RH_FOOT","Gripper_Point"};


};

ModelSettings loadModelSettings(const std::string& filename, const std::string& fieldName = "model_settings", bool verbose = "true");

} // namespace legged_robot
} // namespace ocs2