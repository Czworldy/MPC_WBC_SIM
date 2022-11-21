#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "QuadraticEndEffectorPositionCostCppAd.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticEndEffectorPositionCostCppAd::QuadraticEndEffectorPositionCostCppAd(matrix_t Q,
                                              const PinocchioInterface& pinocchioInterface,
                                              const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                              std::string endEffectorId,
                                              size_t stateDim, size_t inputDim, const std::string& modelName,
                                              const std::string& modelFolder, bool recompileLibraries,
                                              bool verbose)
    : Q_(std::move(Q.cast<ad_scalar_t>())),
      endEffectorId_(std::move(endEffectorId)){
  
  endEffectorFrameId_ = pinocchioInterface.getModel().getBodyId(endEffectorId_);
  
  // initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // set pinocchioInterface to mapping
  std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
  mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  // cost Ad
  auto costAd = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    assert(x.rows() == stateDim + 3);
    const ad_vector_t state = x.segment(0, stateDim);
    const ad_vector_t desiredEEPosition = x.tail(3);
    y = ad_vector_t(1);
    y(0) = quadraticEndEffectorPositionDeviationCppAd(state, desiredEEPosition, pinocchioInterfaceCppAd, *mappingPtr);
  };
  adInterfacePtr_.reset(new ocs2::CppAdInterface(costAd, stateDim + 3, modelName + "_quadraticEndEffectorPositionCost", modelFolder));
  
  if (recompileLibraries) {
    adInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::Second, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::Second, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticEndEffectorPositionCostCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                    const TargetTrajectories& targetTrajectories, const PreComputation&) const { 
  vector_t tapedStateEEPosition(state.rows() + 3);
  tapedStateEEPosition << state, targetTrajectories.getDesiredEEPosition(time);
  return adInterfacePtr_->getFunctionValue(tapedStateEEPosition)(0);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation QuadraticEndEffectorPositionCostCppAd::getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                                                      const TargetTrajectories& targetTrajectories,
                                                                                                      const PreComputation&) const {
  ScalarFunctionQuadraticApproximation cost;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  vector_t tapedStateEEposition(stateDim + 3);
  tapedStateEEposition << state, targetTrajectories.getDesiredEEPosition(time); 

  cost.f = adInterfacePtr_->getFunctionValue(tapedStateEEposition)(0);

  const matrix_t J = adInterfacePtr_->getJacobian(tapedStateEEposition);
  cost.dfdx = J.middleCols(0, stateDim).transpose();
  cost.dfdu = vector_t(inputDim).setZero();

  const matrix_t H = adInterfacePtr_->getHessian(0, tapedStateEEposition);
  cost.dfdxx = H.block(0, 0, stateDim, stateDim);
  cost.dfdux = matrix_t(inputDim, stateDim).setZero();
  cost.dfduu = matrix_t(inputDim, inputDim).setZero();

  return cost;                                                                                     
}
                                                                                                  
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_scalar_t QuadraticEndEffectorPositionCostCppAd::quadraticEndEffectorPositionDeviationCppAd(const ad_vector_t& state, const ad_vector_t& desiredEEPosition,
                                                                                              PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                                              PinocchioStateInputMapping<ad_scalar_t>& mapping) {
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  ad_vector_t q = mapping.getPinocchioJointPosition(state);

  // q.head(6) = ad_vector_t::Zero(6);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  const auto currentPosition(data.oMf[endEffectorFrameId_].translation());
  // for (int i; i < currentPosition.rows(); i++){
  //   std::cerr << "[QuadraticEndEffectorPositionCostCppAd]: " << "Current position is " << currentPosition[i] << "\n";
  // }
  
  ad_vector_t desiredPosition(desiredEEPosition);
  ad_vector_t positionDeviation(currentPosition - desiredPosition);
  ad_vector_t costValue;
  costValue = positionDeviation.transpose() * Q_ * positionDeviation;

  return costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticEndEffectorPositionCostCppAd::QuadraticEndEffectorPositionCostCppAd(const QuadraticEndEffectorPositionCostCppAd& rhs)
    : StateInputCost(rhs), 
      adInterfacePtr_(new CppAdInterface(*rhs.adInterfacePtr_)),
      Q_(std::move(rhs.Q_)),
      endEffectorId_(rhs.endEffectorId_),
      endEffectorFrameId_(rhs.endEffectorFrameId_){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticEndEffectorPositionCostCppAd* QuadraticEndEffectorPositionCostCppAd::clone() const {
  return new QuadraticEndEffectorPositionCostCppAd(*this);
}

} // namespace ocs2