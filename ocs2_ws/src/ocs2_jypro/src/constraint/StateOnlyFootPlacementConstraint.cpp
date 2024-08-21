#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/constraint/StateOnlyFootPlacementConstraint.h"

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
                                                                  size_t contactPointIndex)
    : StateConstraint(ConstraintOrder::Linear),
    referenceManagerPtr_(&referenceManager),
    // endEffectorKinematics_(cast<PinocchioEndEffectorKinematicsCppAd>(endEffectorKinematics)),
    endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
    contactPointIndex_(contactPointIndex),
    transitionSpline_({0.0, 5.*0.35/3., -5.}, {0.35, 0.0, 0.0}) 
    {
      endEffectorKinematics_ = dynamic_cast<PinocchioEndEffectorKinematicsCppAd*>(endEffectorKinematicsPtr_.get());
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateOnlyFootPlacementConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  // return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  // return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateOnlyFootPlacementConstraint::getNumConstraints(scalar_t time) const {
  throw std::runtime_error("[StateOnlyFootPlacementConstraint] getNumConstraints not implemented!");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateOnlyFootPlacementConstraint::getValue(scalar_t time, const vector_t& state,
                                                    const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  vector_t tapedTimeState(1 + state.rows());

  tapedTimeState << time, state;

  FootConstraints footConstraint = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  // vector_t b = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  vector_t f = footConstraint.A * endEffectorKinematicsPtr_->getPosition(state).front() + footConstraint.b;



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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
  constraint.f = footConstraint.A * endEffectorKinematicsPtr_->getPosition(state).front() + footConstraint.b;

  scalar_t s_t(0.);
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);
  
  if(referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]){
    s_t = 0;
  }
  else{
    s_t = transitionSpline_.position(0.35 - swingTimeLeft);
  }

  constraint.f.noalias() += s_t * (footConstraint.A).rowwise().norm();

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

  // const matrix_t J = footConstraint.A * eePosition.dfdx;
  // constraint.dfdx =  J.rightCols(stateDim);
  constraint.dfdx =  footConstraint.A * eePosition.dfdx;

  return constraint;                                                        
}


} // namespace legged_robot
} // namespace ocs2