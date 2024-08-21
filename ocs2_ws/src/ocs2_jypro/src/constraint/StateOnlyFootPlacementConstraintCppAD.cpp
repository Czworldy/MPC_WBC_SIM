#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/constraint/StateOnlyFootPlacementConstraintCppAD.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_jypro/LeggedRobotPreComputation.h"
#include "ocs2_jypro/foot_planner/FootConstraintsPlanner.h"


namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateOnlyFootPlacementConstraint::StateOnlyFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   const std::string& modelName, const std::string& modelFolder, bool recompile,
                                                                   Config config, size_t contactPointIndex,
                                                                   const size_t& stateDim)
    : StateConstraintCppAd(ConstraintOrder::Linear),
    referenceManagerPtr_(&referenceManager),
    endEffectorKinematics_(cast<PinocchioEndEffectorKinematicsCppAd>(endEffectorKinematics)),
    config_(std::move(config)),
    contactPointIndex_(contactPointIndex),
    stateDim_(stateDim),
    transitionSpline_({0.0, 5.*0.35/3., -5.}, {0.35, 0.0, 0.0}) {

        // initialize CppAD interface
        auto pinocchioInterfaceCppAd = endEffectorKinematics_.pinocchioInterface_.toCppAd();

        // position function
        positionFunc_ = [&, this](const ad_vector_t& x, ad_vector_t& y) {
          endEffectorKinematics_.updateCallback_(x, pinocchioInterfaceCppAd);
          y = getPositionCppAd(pinocchioInterfaceCppAd, *endEffectorKinematics_.mappingPtr, x);
        };

        // start build CppAD interface
        initialize(stateDim_, 0, modelName, modelFolder, recompile, true);
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateOnlyFootPlacementConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  // return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  // return true;
}

size_t StateOnlyFootPlacementConstraint::getNumConstraints(scalar_t time) const {
  throw std::runtime_error("[StateOnlyFootPlacementConstraint] getNumConstraints not implemented!");
}

ad_vector_t StateOnlyFootPlacementConstraint::constraintFunction(ad_scalar_t time, const ad_vector_t& state, 
                                            const ad_vector_t& parameters) const{
  ad_vector_t y = ad_vector_t::Zero(3);
  positionFunc_(state, y);
  return y;
}

vector_t StateOnlyFootPlacementConstraint::getValue(scalar_t time, const vector_t& state,
                                                    const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  vector_t tapedTimeState(1 + state.rows());

  tapedTimeState << time, state;

  FootConstraints footConstraint = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  // vector_t b = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  vector_t f = footConstraint.A * getCppAdInterface()->getFunctionValue(tapedTimeState, vector_t(0)) + footConstraint.b;



  scalar_t s_t(0.);
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);

  if(referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]){
    // f.array() += stance_tol;
    s_t = 0;
  }
  else{
    s_t = transitionSpline_.position(0.35 - swingTimeLeft);
  }

  f.noalias() += s_t * (footConstraint.A).rowwise().norm();

  return f;
}

VectorFunctionLinearApproximation StateOnlyFootPlacementConstraint::getLinearApproximation(
                                                        scalar_t time, const vector_t& state, 
                                                        const PreComputation& preComp) const{
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = vector_t(0);
  vector_t tapedTimeState(1 + stateDim);

  tapedTimeState << time, state;

  FootConstraints footConstraint = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  constraint.f = footConstraint.A * getCppAdInterface()->getFunctionValue(tapedTimeState, vector_t(0)) + footConstraint.b;

  scalar_t s_t(0.);
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);
  
  if(referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]){
    s_t = 0;
  }
  else{
    s_t = transitionSpline_.position(0.35 - swingTimeLeft);
  }

  constraint.f.noalias() += s_t * (footConstraint.A).rowwise().norm();

  const matrix_t J = footConstraint.A * getCppAdInterface()->getJacobian(tapedTimeState, params);
  constraint.dfdx =  J.rightCols(stateDim);

  return constraint;                                                        
}

// if getOrder == ConstraintOrder::Linear using Quadratic Approximation Strategy, this function will not be called.
VectorFunctionQuadraticApproximation StateOnlyFootPlacementConstraint::getQuadraticApproximation(
                                                                      scalar_t time, const vector_t& state,
                                                                      const PreComputation&preComp) const {
  if (getOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateOnlyFootPlacementConstraint] Quadratic approximation not supported!");
  }

  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = vector_t(0);
  vector_t tapedTimeState(1 + stateDim);

  tapedTimeState << time, state;

  FootConstraints footConstraint = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  constraint.f = footConstraint.A * getCppAdInterface()->getFunctionValue(tapedTimeState, vector_t(0)) + footConstraint.b;

  scalar_t s_t(0.);
  
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);

  if(referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]){
    s_t = 0;
  }
  else{
    s_t = transitionSpline_.position(0.35 - swingTimeLeft);
  }

  constraint.f.noalias() += s_t * (footConstraint.A).rowwise().norm();
  // std::cout << "b:" << b.transpose() << "\t time:" << time << "\t leg:" 
  //   << contactPointIndex_<< "\t f:"<< constraint.f.transpose() << std::endl;

  // std::cout << "cppad:" << f.transpose() << "\n";
  // std::cout << "y: " << state(7) << "\n";
  const matrix_t J = footConstraint.A * getCppAdInterface()->getJacobian(tapedTimeState, params);
  constraint.dfdx =  J.rightCols(stateDim);

  const size_t numCppadOut = 3;

  matrix_array_t H;
  H.resize(numCppadOut);

  for (int i = 0; i < numCppadOut; i++) {
    const matrix_t H_i = getCppAdInterface()->getHessian(i, tapedTimeState, params);
    H[i] = H_i.bottomRightCorner(stateDim, stateDim);
  }

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++){
    constraint.dfdxx[i].noalias() = footConstraint.A.row(i)[0] * H[0] + footConstraint.A.row(i)[1] * H[1] + footConstraint.A.row(i)[2] * H[2];
    constraint.dfdxx[i].diagonal().array() -= config_.hessianDiagonalShift;

  }
  

  return constraint;
}

ad_vector_t StateOnlyFootPlacementConstraint::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state) {
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

} // namespace legged_robot
} // namespace ocs2