#include "LeggedRobotWithArmDynamicsAD.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmDynamicsAD::LeggedRobotWithArmDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info,
                                                           const std::string& modelName, const ModelSettings& modelSettings)
    : pinocchioCentroidalDynamicsAd_(pinocchioInterface, info, modelName, modelSettings.modelFolderCppAd, 
                                     modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotWithArmDynamicsAD::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) {
    return pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LeggedRobotWithArmDynamicsAD::linearApproximation(scalar_t time, const vector_t& state, const vector_t& input, 
                                                                                    const PreComputation& preComp) {
    return pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
}

} // namespace legged_robot
} // namespace ocs2