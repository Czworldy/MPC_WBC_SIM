#include "LeggedRobotWithArmStateInputQuadraticCost.h"

#include <utils.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmStateInputQuadraticCost::LeggedRobotWithArmStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info,
                                                                                     const SwitchedModelReferenceManager& referenceManager)
        : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmStateInputQuadraticCost* LeggedRobotWithArmStateInputQuadraticCost::clone() const {
    return new LeggedRobotWithArmStateInputQuadraticCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, vector_t> LeggedRobotWithArmStateInputQuadraticCost::getStateInputDeviation(
        scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories) const {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
    return {state - xNominal, input - uNominal};
}


} // namespace legged_robot
} // namespace ocs2