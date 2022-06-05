#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

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
    // endEffectorKinematics(static_cast<std::shared_ptr<PinocchioEndEffectorKinematicsCppAd>>(endEffectorKinematics.clone())),
    // eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 6)),
    endEffectorKinematics_(cast<PinocchioEndEffectorKinematicsCppAd>(endEffectorKinematics)),
    config_(std::move(config)),
    contactPointIndex_(contactPointIndex),
    stateDim_(stateDim) {

        // initialize CppAD interface
        auto pinocchioInterfaceCppAd = endEffectorKinematics_.pinocchioInterface_.toCppAd();

        for(const auto& i:endEffectorKinematics_.getendEffectorFrameIds()){
          std::cout << "id:" << i << "\n";
        }
        //  set pinocchioInterface to mapping
        // std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(endEffectorKinematics_.mapping_.clone());//段错误
        // mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

        // position function
        positionFunc_ = [&, this](const ad_vector_t& x, ad_vector_t& y) {
          endEffectorKinematics_.updateCallback_(x, pinocchioInterfaceCppAd);
          y = getPositionCppAd(pinocchioInterfaceCppAd, *endEffectorKinematics_.mappingPtr, x);
        };

        scalar_t tor = 0.05;
        vector_t B_veclf = vector_t::Zero(6);
        vector_t B_vecrf = vector_t::Zero(6);
        vector_t B_veclh = vector_t::Zero(6);
        vector_t B_vecrh = vector_t::Zero(6);
        vector_t bias = tor * vector_t::Ones(6);
        B_veclf << 0.177, -0.177, -0.338, 0.338, 0.1, 0.1;
        B_vecrf << -0.177, 0.177, -0.338, 0.338, 0.1, 0.1;
        B_veclh << 0.177, -0.177, 0.322, -0.322, 0.1, 0.1;
        B_vecrh << -0.177, 0.177, 0.322, -0.322, 0.1, 0.1;

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
        initialize(stateDim_, 0, modelName, "/tmp/ocs2",true, true);
        
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateOnlyFootPlacementConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  // return true;
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

  vector_t state_ = state;
  vector_t y = vector_t::Zero(6);
  y(2) = state(8);
  state_.segment<6>(6) = y;
  tapedTimeState << time, state;

  vector_t b = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  vector_t f = Ax * getCppAdInterface()->getFunctionValue(tapedTimeState, getParameters(time)) + b;

  scalar_t s_t(0.);
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);

  s_t = 3 * std::pow(swingTimeLeft, 2);

  f.noalias() += s_t * vector_t::Ones(getNumConstraints(time));

  return f;
}

VectorFunctionLinearApproximation StateOnlyFootPlacementConstraint::getLinearApproximation(
                                                        scalar_t time, const vector_t& state, 
                                                        const PreComputation& preComp) const{
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = getParameters(time);
  vector_t tapedTimeState(1 + stateDim);
  vector_t state_ = state;
  vector_t y = vector_t::Zero(6);
  y(2) = state(8);
  state_.segment<6>(6) = y;
  tapedTimeState << time, state;

  vector_t b = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  constraint.f = Ax * getCppAdInterface()->getFunctionValue(tapedTimeState, params) + b;
  scalar_t s_t(0.);
  
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);
  s_t = 3 * std::pow(swingTimeLeft, 2);

  constraint.f.noalias() += s_t * vector_t::Ones(getNumConstraints(time));

  const matrix_t J = Ax * getCppAdInterface()->getJacobian(tapedTimeState, params);
  constraint.dfdx =  J.rightCols(stateDim);

  return constraint;
                                                        
  }
VectorFunctionQuadraticApproximation StateOnlyFootPlacementConstraint::getQuadraticApproximation(
                                                                      scalar_t time, const vector_t& state,
                                                                      const PreComputation&preComp) const {
  if (getOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateOnlyFootPlacementConstraint] Quadratic approximation not supported!");
  }

  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = getParameters(time);
  vector_t tapedTimeState(1 + stateDim);
  vector_t state_ = state;
  vector_t y = vector_t::Zero(6);
  y(2) = state(8);
  state_.segment<6>(6) = y;
  tapedTimeState << time, state;

  
  vector_t b = preCompLegged.getFootPlacementConstraint()[contactPointIndex_];
  // std::cout << "b:" << b.transpose() << "\t time:" << time << "\t leg:" << contactPointIndex_ << std::endl;
  vector_t f = getCppAdInterface()->getFunctionValue(tapedTimeState, params);
  constraint.f = Ax * f + b;
  scalar_t s_t(0.);
  
  scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);
  s_t = 3 * std::pow(swingTimeLeft, 2);

  constraint.f.noalias() += s_t * vector_t::Ones(getNumConstraints(time));
  std::cout << "b:" << b.transpose() << "\t time:" << time << "\t leg:" 
    << contactPointIndex_<< "\t f:"<< constraint.f.transpose() << std::endl;

  std::cout << "cppad:" << f.transpose() << "\n";


  // std::cout << "y: " << state(7) << "\n";


  const matrix_t J = Ax * getCppAdInterface()->getJacobian(tapedTimeState, params);
  constraint.dfdx =  J.rightCols(stateDim);

  // const size_t numConstraints = constraint.f.rows();
  // constraint.dfdxx.resize(numConstraints);
  // constraint.dfdux.resize(numConstraints);
  // constraint.dfduu.resize(numConstraints);
  // for (int i = 0; i < numConstraints; i++) {
  //   const matrix_t H = getCppAdInterface()->getHessian(i, tapedTimeState, params);
  //   constraint.dfdxx[i] = H.bottomRightCorner(stateDim, stateDim);
  //   constraint.dfdxx[i].diagonal().array() -= config_.hessianDiagonalShift;
  // }

  // std::cout << "Constraints time: " << time << "\n";

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
    constraint.dfdxx[i].noalias() = Ax.row(i)[0] * H[0] + Ax.row(i)[1] * H[1] + Ax.row(i)[2] * H[2];
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
//重写三个get

} // namespace legged_robot
} // namespace ocs2