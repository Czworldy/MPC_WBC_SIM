#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/constraint/CBFFootPlacementConstraintCppAD.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_jypro/LeggedRobotPreComputation.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"



namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CBFFootPlacementConstraint::CBFFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   const CentroidalModelInfo& info,
                                                                   const std::string& modelName,
                                                                   Config config, size_t contactPointIndex)
    : StateInputConstraintCppAd(ConstraintOrder::Quadratic),
    referenceManagerPtr_(&referenceManager),
    endEffectorKinematics_(cast<PinocchioEndEffectorKinematicsCppAd>(endEffectorKinematics)),
    info_(info),
    config_(std::move(config)),
    contactPointIndex_(contactPointIndex){

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

        JacobiFunc_ = [&, this](const ad_vector_t& x, ad_matrix_t& J) {
          endEffectorKinematics_.updateCallback_(x, pinocchioInterfaceCppAd);
          J = getJacobiCppAd(pinocchioInterfaceCppAd, *endEffectorKinematics_.mappingPtr, x);
        };

        systemFlowMapFunc_ = [&](const ad_vector_t& x, ad_vector_t& y) {

          // mapping
          CentroidalModelPinocchioMappingCppAd mappingCppAd(info_.toCppAd());
          mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);

          ad_vector_t state = x.head(info_.stateDim);
          ad_vector_t input = x.tail(info_.inputDim);
          y = getValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
        };

        scalar_t tor = 0.05;
        vector_t B_veclf = vector_t::Zero(4);
        vector_t B_vecrf = vector_t::Zero(4);
        vector_t B_veclh = vector_t::Zero(4);
        vector_t B_vecrh = vector_t::Zero(4);
        vector_t bias = tor * vector_t::Ones(4);
        B_veclf << 0.177, -0.177, -0.338, 0.338;
        B_vecrf << -0.177, 0.177, -0.338, 0.338;
        B_veclh << 0.177, -0.177, 0.322, -0.322;
        B_vecrh << -0.177, 0.177, 0.322, -0.322;

        B_veclf = (B_veclf + bias).eval(); 
        B_vecrf = (B_vecrf + bias).eval(); 
        B_veclh = (B_veclh + bias).eval(); 
        B_vecrh = (B_vecrh + bias).eval(); 

        B << B_veclf, B_vecrf, B_veclh, B_vecrh;

        Ax <<    1, 0, 0,
                -1, 0, 0,
                0, 1, 0,
                0, -1, 0;
              
        gamma = 20.;
        dsdt = 1.2;
        // std::cout << "initialize CBFFootPlacementConstraint\n";
        initialize(info.stateDim, info.inputDim, 0, modelName, "/tmp/ocs2", true, true);
        // std::cout << "initialize CBFFootPlacementConstraint asdasdasd\n";

        
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CBFFootPlacementConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  // return true;
}


ad_vector_t CBFFootPlacementConstraint::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                            const ad_vector_t& parameters) const{
  ad_vector_t y(3), xdot(info_.stateDim), f(4);
  ad_matrix_t J(3, info_.stateDim);
  positionFunc_(state, y);
  JacobiFunc_(state, J);
  ad_vector_t stateinput = (ad_vector_t(info_.stateDim + info_.inputDim) << state, input).finished();
  systemFlowMapFunc_(stateinput, xdot);
  f.noalias() = Ax.cast<ad_scalar_t>() * J * xdot +  ad_scalar_t(gamma) * Ax.cast<ad_scalar_t>() * y;
  return f;
}

vector_t CBFFootPlacementConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                    const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  vector_t tapedTimeStateInput(1 + state.rows() + input.rows());
  vector_t state_ = state;
  vector_t y = vector_t::Zero(6);

  y(2) = state(8);
  state_.segment<6>(6) = y;
  tapedTimeStateInput << time, state_, input;

  vector_t b = B.col(contactPointIndex_);
  vector_t f = getCppAdInterface()->getFunctionValue(tapedTimeStateInput, vector_t(0));
  // std::cout << "cppad:" << f.transpose() << "\n";

  scalar_t s_t(0.);
  const scalar_t swingTimeLeft(preCompLegged.getSwingTimeLeft()[contactPointIndex_]);

  // if(!referenceManagerPtr_->getContactFlags(time)[contactPointIndex_])
  s_t = dsdt * std::pow(swingTimeLeft, 2);
  // assert(s_t >= 0);
                          
  f.noalias() += gamma*(s_t * vector_t::Ones(getNumConstraints(time)) + b) - 2*dsdt*swingTimeLeft*vector_t::Ones(getNumConstraints(time));
  std::cout << "f:" << f.transpose() << "\n";
  return f;
}

VectorFunctionLinearApproximation CBFFootPlacementConstraint::getLinearApproximation(
                                                        scalar_t time, const vector_t& state, const vector_t& input,
                                                        const PreComputation& preComp) const{
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = vector_t(0);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);

  vector_t state_ = state;
  vector_t y = vector_t::Zero(6);
  y(2) = state(8);
  state_.segment<6>(6) = y;
  tapedTimeStateInput << time, state_, input;

  constraint.f = getValue(time, state, input, preComp);

  const matrix_t J = getCppAdInterface()->getJacobian(tapedTimeStateInput, params);
  constraint.dfdx = J.middleCols(1, stateDim);
  constraint.dfdu = J.rightCols(inputDim);

  return constraint;
                                                        
  }
VectorFunctionQuadraticApproximation CBFFootPlacementConstraint::getQuadraticApproximation(
                                                                  scalar_t time, const vector_t& state, const vector_t& input,
                                                                  const PreComputation&preComp) const {
  if (getOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[CBFFootPlacementConstraint] Quadratic approximation not supported!");
  }

  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = vector_t(0);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  
  vector_t state_ = state;
  vector_t y = vector_t::Zero(6);
  y(2) = state(8);
  state_.segment<6>(6) = y;
  tapedTimeStateInput << time, state_, input;

  constraint.f = getValue(time, state, input, preComp);

  const matrix_t J = getCppAdInterface()->getJacobian(tapedTimeStateInput, params);
  constraint.dfdx = J.middleCols(1, stateDim);
  constraint.dfdu = J.rightCols(inputDim);

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++) {
    const matrix_t H = getCppAdInterface()->getHessian(i, tapedTimeStateInput, params);
    constraint.dfdxx[i] = H.block(1, 1, stateDim, stateDim);
    constraint.dfdux[i] = H.block(1 + stateDim, 1, inputDim, stateDim);
    constraint.dfduu[i] = H.bottomRightCorner(inputDim, inputDim);
  }

  return constraint;
}

ad_vector_t CBFFootPlacementConstraint::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
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

ad_matrix_t CBFFootPlacementConstraint::getJacobiCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state) {
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;


  pinocchio::forwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);

  std::vector<ad_matrix_t> Js;
  for (const auto& frameId : endEffectorKinematics_.getendEffectorFrameIds()) {
    ad_matrix_t J = ad_matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    ad_matrix_t dfdx(3,info_.stateDim);
    dfdx.leftCols(6) = ad_matrix_t::Zero(3,6);
    dfdx.rightCols(model.nq) = J.topRows<3>();
    // std::tie(dfdx, std::ignore) = endEffectorKinematics_.mappingPtr->getOcs2Jacobian(state, J.topRows<3>(), ad_matrix_t::Zero(0, model.nv));
    Js.emplace_back(std::move(dfdx));
  }

  return Js.front();
}

ad_vector_t CBFFootPlacementConstraint::getValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                         const CentroidalModelPinocchioMappingCppAd& mappingCppAd, const ad_vector_t& state,
                                                         const ad_vector_t& input) {
  const auto& info = mappingCppAd.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());

  const ad_vector_t qPinocchio = mappingCppAd.getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterfaceCppAd, info, qPinocchio);

  ad_vector_t stateDerivative(info.stateDim);

  // compute center of mass acceleration and derivative of the normalized angular momentum
  centroidal_model::getNormalizedMomentum(stateDerivative, info) =
      getNormalizedCentroidalMomentumRate(pinocchioInterfaceCppAd, info, input);

  // derivatives of the floating base variables + joint velocities
  centroidal_model::getGeneralizedCoordinates(stateDerivative, info) = mappingCppAd.getPinocchioJointVelocity(state, input);

  return stateDerivative;
}

} // namespace legged_robot
} // namespace ocs2