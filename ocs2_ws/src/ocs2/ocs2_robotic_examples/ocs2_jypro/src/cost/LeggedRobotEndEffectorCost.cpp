#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/cost/LeggedRobotEndEffectorCost.h"
#include "ocs2_jypro/LeggedRobotPreComputation.h"


namespace ocs2 {
namespace legged_robot {

LeggedRobotEndEffectorCost::LeggedRobotEndEffectorCost(matrix_t Q, matrix_t R, 
                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                       size_t contactPointIndex,size_t stateDim, size_t inputDim,
                                                       const std::string& modelName, const std::string& modelFolderCppAd,
                                                       bool recompileCppAd) 
  : Q_(std::move(Q)), R_(std::move(R)), 
  endEffectorKinematics_(cast<PinocchioEndEffectorKinematicsCppAd>(endEffectorKinematics)),
  // pinocchioInterfaceCppAd_(endEffectorKinematics_.pinocchioInterface_.toCppAd()),
  contactPointIndex_(contactPointIndex) {
    
    auto pinocchioInterfaceCppAd_ = endEffectorKinematics_.pinocchioInterface_.toCppAd();
    endEffectorKinematics_.mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd_);
    positionFunc_ = [&, this](const ad_vector_t& x, ad_vector_t& y) {
      endEffectorKinematics_.updateCallback_(x, pinocchioInterfaceCppAd_);
      y = getPositionCppAd(pinocchioInterfaceCppAd_, *(endEffectorKinematics_.mappingPtr), x);
    };

      // velocity function
    velocityFunc_ = [&, this](const ad_vector_t& x, ad_vector_t& y) {
      const ad_vector_t state = x.head(stateDim);
      const ad_vector_t input = x.tail(inputDim);
      endEffectorKinematics_.updateCallback_(state, pinocchioInterfaceCppAd_);
      y = getVelocityCppAd(pinocchioInterfaceCppAd_, *(endEffectorKinematics_.mappingPtr), state, input);
     };

    initialize(stateDim, inputDim, 6, modelName, modelFolderCppAd, recompileCppAd, true);
}

LeggedRobotEndEffectorCost* LeggedRobotEndEffectorCost::clone() const {
  return new LeggedRobotEndEffectorCost(*this);
}

ad_vector_t LeggedRobotEndEffectorCost::costVectorFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, 
                                                     const ad_vector_t& parameters) const {
  ad_vector_t position, velocity, x(state.rows() + input.rows());
  ad_vector_t cost(6);
  positionFunc_(state, position);
  x << state, input;
  velocityFunc_(x, velocity);

  // cost = ocs2::ad_scalar_t(0.5) * (position - parameters.head(3)).dot(Q_.cast<ad_scalar_t>() * (position - parameters.head(3)));
  // cost += ocs2::ad_scalar_t(0.5) * (velocity - parameters.tail(3)).dot(R_.cast<ad_scalar_t>() * (velocity - parameters.tail(3)));
  cost.head(3) = position - parameters.head(3);
  cost.tail(3) = velocity - parameters.tail(3);
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t LeggedRobotEndEffectorCost::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {
  vector_t timeStateInput(1 + state.rows() + input.rows());
  timeStateInput << time, state, input;
  const auto parameters = getParameters(time, targetTrajectories, preComputation);
  const auto costVector = adInterfacePtr_->getFunctionValue(timeStateInput, parameters);
  // return 0.5 * costVector.squaredNorm();
  return 0.5 * (costVector.head(3).dot(Q_ * costVector.head(3)) + costVector.tail(3).dot(R_ * costVector.tail(3)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation LeggedRobotEndEffectorCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const TargetTrajectories& targetTrajectories,
                                                                                            const PreComputation& preComputation) const {
  const auto stateDim = state.rows();
  const auto inputDim = input.rows();
  vector_t timeStateInput(1 + stateDim + inputDim);
  timeStateInput << time, state, input;
  const auto parameters = getParameters(time, targetTrajectories, preComputation);
  const auto J = adInterfacePtr_->getJacobian(timeStateInput, parameters);
  const auto costVector = adInterfacePtr_->getFunctionValue(timeStateInput, parameters);


  ScalarFunctionQuadraticApproximation L;
  L.f = getValue(time, state, input, targetTrajectories, preComputation); //use override.
  matrix_t W = matrix_t::Zero(6, 6);
  W.topLeftCorner(3, 3) = Q_; W.bottomRightCorner(3, 3) = R_;
  const auto dfdx_dfdu = J.transpose() * W * costVector;
  const auto approxHessian = J.transpose() * W * J;
  L.dfdx.noalias() = dfdx_dfdu.middleRows(1, stateDim);
  L.dfdu.noalias() = dfdx_dfdu.bottomRows(inputDim);
  L.dfdxx = approxHessian.block(1, 1, stateDim, stateDim);
  L.dfdux.noalias() = approxHessian.block(1 + stateDim, 1, inputDim, stateDim);
  L.dfduu.noalias() = approxHessian.block(1 + stateDim, 1 + stateDim, inputDim, inputDim);
  return L;
}



vector_t LeggedRobotEndEffectorCost::getParameters(scalar_t time, const TargetTrajectories& targetTrajectories,
                                 const PreComputation& preComputation ) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComputation);
  return preCompLegged.getEEReference()[contactPointIndex_];
}

// scalar_t LeggedRobotEndEffectorCost::getValue(scalar_t time, const vector_t& state, const vector_t& input, 
//                                               const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {
//   const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComputation);
//   const vector_t parameters = preCompLegged.getEEReference()[contactPointIndex_];

//   const vector_t position = endEffectorKinematics_.getPosition(state);
//   const vector_t velocity = endEffectorKinematics_.getVelocity(state, input);
//   scalar_t cost = 0.0;
//   cost += 0.5 * (position - parameters.head(3)).dot(Q_ * (position - parameters.head(3)));
//   cost += 0.5 * (velocity - parameters.tail(3)).dot(R_ * (velocity - parameters.tail(3)));
//   return cost;
// }

// ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
//                                                                  const TargetTrajectories& targetTrajectories,
//                                                                  const PreComputation& preComputation) const {

//   ScalarFunctionQuadraticApproximation cost;
//   const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComputation);        
//   const vector_t parameters = preCompLegged.getEEReference()[contactPointIndex_];

//   cost.f = getValue(time, state, input, targetTrajectories, preComputation);

//   cost.dfdx = endEffectorKinematics_.getPositionLinearApproximation(state).front().dfdx.transpose() 
//                 * Q_ * (endEffectorKinematics_.getPosition(state) - parameters.head(3));
//   cost.dfdu = endEffectorKinematics_.getVelocityLinearApproximation(state, input).front().dfdu.transpose() 
//                 * R_ * (endEffectorKinematics_.getVelocity(state, input) - parameters.tail(3));

// }
ad_vector_t LeggedRobotEndEffectorCost::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state) const {
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t positions(3 * endEffectorKinematics_.getendEffectorFrameIds().size());
  for (int i = 0; i < endEffectorKinematics_.getendEffectorFrameIds().size(); i++) {
    const size_t frameId = endEffectorKinematics_.getendEffectorFrameIds()[i];
    positions.segment<3>(3 * i) = data.oMf[frameId].translation();
  }
  return positions;
}

ad_vector_t LeggedRobotEndEffectorCost::getVelocityCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state, const ad_vector_t& input) const {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);
  const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t velocities(3 * endEffectorKinematics_.getendEffectorFrameIds().size());
  for (int i = 0; i < endEffectorKinematics_.getendEffectorFrameIds().size(); i++) {
    const size_t frameId = endEffectorKinematics_.getendEffectorFrameIds()[i];
    velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
  }
  return velocities;
}

}  // namespace legged_robot
}  // namespace ocs2
