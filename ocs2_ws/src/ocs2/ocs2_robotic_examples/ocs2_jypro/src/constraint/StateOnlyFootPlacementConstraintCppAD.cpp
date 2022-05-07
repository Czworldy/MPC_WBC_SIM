#include "ocs2_jypro/constraint/StateOnlyFootPlacementConstraintCppAD.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_jypro/LeggedRobotPreComputation.h"


namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateOnlyFootPlacementConstraint::StateOnlyFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   const std::string& modelName,
                                                                   Config config, size_t contactPointIndex,
                                                                   const size_t& stateDim)
    : StateConstraintCppAd(ConstraintOrder::Quadratic),
    referenceManagerPtr_(&referenceManager),
    endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
    // eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 6)),
    config_(std::move(config)),
    contactPointIndex_(contactPointIndex),
    stateDim_(stateDim) {

      std::cout << "StateOnlyFootPlacementConstraint::StateOnlyFootPlacementConstraint" << std::endl;

        // eeLinearConstraintPtr_.reset(new EndEffectorLinearConstraint(endEffectorKinematics, 6, conf));
        size_t tor = 0.03;
        vector_t B_veclf = vector_t::Zero(6);
        vector_t B_vecrf = vector_t::Zero(6);
        vector_t B_veclh = vector_t::Zero(6);
        vector_t B_vecrh = vector_t::Zero(6);
        vector_t bias = tor * vector_t::Ones(6);
        B_veclf << 0.2, -0.2, -0.32, 0.32, 0.0, 0.0;
        B_vecrf << -0.15, 0.15, -0.36, 0.36, 0.0, 0.0;
        B_veclh << 0.15, -0.15, 0.31, -0.31, 0.0, 0.0;
        B_vecrh << -0.2, 0.2, 0.3, -0.3, 0.0, 0.0;

        B_veclf = (B_veclf + bias).eval(); 
        B_vecrf = (B_vecrf + bias).eval(); 
        B_veclh = (B_veclh + bias).eval(); 
        B_vecrh = (B_vecrh + bias).eval(); 

        B << B_veclf, B_vecrf, B_veclh, B_vecrh;

        Ax <<    1, 0, 0,
                -1, 0, 0,
                0, 1, 0,
                0, -1, 0,
                0, 0, 1,
                0, 0, -1;
      std::cout << "StateOnlyFootPlacementConstraint::initialize" << std::endl;
      if(endEffectorKinematicsPtr_->positionFunc == nullptr)
        std::cout << "fuck!";
        initialize(stateDim_, 0, modelName, "/tmp/ocs2",true,true);
        
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateOnlyFootPlacementConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}


ad_vector_t StateOnlyFootPlacementConstraint::constraintFunction(ad_scalar_t time, const ad_vector_t& state, 
                                            const ad_vector_t& parameters) const{
  ad_vector_t y = ad_vector_t::Zero(3);
  endEffectorKinematicsPtr_->positionFunc(state, y);
  // return Ax.cast<ad_scalar_t>() * y;
  return y;
}

vector_t StateOnlyFootPlacementConstraint::getValue(scalar_t time, const vector_t& state,
                                                    const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  vector_t tapedTimeState(1 + state.rows());
  tapedTimeState << time, state;

  vector_t b = B.col(contactPointIndex_);
  vector_t f = getCppAdInterface()->getFunctionValue(tapedTimeState, getParameters(time)) + b;
  scalar_t s_t = preCompLegged.getSwingTimeLeft()[contactPointIndex_];
                          
  f.noalias() += s_t * vector_t::Ones(getNumConstraints(time));

  return f;
}

VectorFunctionLinearApproximation StateOnlyFootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t& state, 
                                                        const PreComputation& preComp) const{
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = getParameters(time);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  vector_t b = B.col(contactPointIndex_);
  constraint.f = getCppAdInterface()->getFunctionValue(tapedTimeState, params) + b;
  scalar_t s_t = preCompLegged.getSwingTimeLeft()[contactPointIndex_];

  constraint.f.noalias() += s_t * vector_t::Ones(getNumConstraints(time));

  const matrix_t J = getCppAdInterface()->getJacobian(tapedTimeState, params);
  constraint.dfdx =  J.rightCols(stateDim);

  return constraint;
                                                        
  }
VectorFunctionQuadraticApproximation StateOnlyFootPlacementConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                      const PreComputation&preComp) const {
  if (getOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateOnlyFootPlacementConstraint] Quadratic approximation not supported!");
  }

  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = getParameters(time);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  vector_t b = B.col(contactPointIndex_);
  constraint.f = getCppAdInterface()->getFunctionValue(tapedTimeState, params) + b;
  scalar_t s_t = preCompLegged.getSwingTimeLeft()[contactPointIndex_];

  constraint.f.noalias() += s_t * vector_t::Ones(getNumConstraints(time));

  const matrix_t J = getCppAdInterface()->getJacobian(tapedTimeState, params);
  constraint.dfdx =  J.rightCols(stateDim);

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++) {
    const matrix_t H = getCppAdInterface()->getHessian(i, tapedTimeState, params);
    constraint.dfdxx[i] = H.bottomRightCorner(stateDim, stateDim);
  }

  return constraint;
}
//重写三个get

} // namespace legged_robot
} // namespace ocs2