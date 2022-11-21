#include <pinocchio/fwd.hpp>

#include "LeggedRobotWithArmQuadraticEEPositionCost.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2 {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmQuadraticEEPositionCost::LeggedRobotWithArmQuadraticEEPositionCost(matrix_t Q,
                                                                                     std::string endEffectorId,
                                                                                     size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                                                                     const std::string& modelFolder, bool recompileLibraries,
                                                                                     bool verbose)
        : Q_(std::move(Q)),
          endEffectorId_(std::move(endEffectorId)){
    std::string urdfString;
    if (!ros::param::get("/legged_robot_with_arm_description", urdfString)) {
      std::cerr << "[LeggedRobotWithArmQuadraticEEPositionCost]Param " << "/legged_robot_with_arm_description" << " not found; unable to generate urdf" << std::endl;
    }
    pinocchioInterfacePtr_.reset(new PinocchioInterface(ocs2::centroidal_model::createPinocchioInterface(urdf::parseURDF(urdfString), modelSettings_.jointNames)));
    endEffectorFrameId_ = pinocchioInterfacePtr_->getModel().getBodyId(endEffectorId_);
    initialize(stateDim, inputDim, parameterDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotWithArmQuadraticEEPositionCost::getParameters(scalar_t time, const TargetTrajectories& targetTrajectories) const {
    return targetTrajectories.getDesiredEEPosition(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_scalar_t LeggedRobotWithArmQuadraticEEPositionCost::costFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, 
                                                                    const ad_vector_t& parameters) const {
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    const ad_vector_t q = state.tail(state.rows() - 6);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t currentPosition(data.oMf[endEffectorFrameId_].translation());

    ad_scalar_t costValue;

    costValue = currentPosition.transpose() * Q_ * currentPosition;

    return costValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmQuadraticEEPositionCost::LeggedRobotWithArmQuadraticEEPositionCost(const LeggedRobotWithArmQuadraticEEPositionCost& other) 
    : StateInputCostCppAd(other){

}


}