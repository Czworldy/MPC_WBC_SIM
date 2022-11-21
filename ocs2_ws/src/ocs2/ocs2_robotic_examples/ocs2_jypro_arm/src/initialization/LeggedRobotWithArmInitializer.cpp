#include "LeggedRobotWithArmInitializer.h"

#include "utils.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmInitializer::LeggedRobotWithArmInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager, 
                                                             bool extendNormalizedMomentum)
        : info_(std::move(info)), referenceManagerPtr_(&referenceManager), extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmInitializer* LeggedRobotWithArmInitializer::clone() const {
    return new LeggedRobotWithArmInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
    const auto& contactFlags = referenceManagerPtr_->getContactFlags(time);
    input = weightCompensatingInput(info_, contactFlags);
    nextState = state;
    if(!extendNormalizedMomentum_) {
        centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
    }
}

} // namespace legged_robot
} // namespace ocs2