/*
 * @Author: Jiyu Yu 
 * @Date: 2024-06-27 18:50:37 
 * @Last Modified by: Jiyu Yu
 * @Last Modified time: 2024-07-02 15:44:55
 */

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "ocs2_wbc/QuadrupedWbc.h"

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

namespace ocs2 {
namespace wbc {

using namespace switched_model;

QuadrupedWbc::QuadrupedWbc(const ocs2::PinocchioInterface &pinocchioInterface,
                           const anymal::QuadrupedPinocchioMapping& quadrupedPinocchioMapping,
                           const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                           std::vector<std::string> endEffectorIds,
                           const std::string& paramFile)
      : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      mapping_(quadrupedPinocchioMapping),
      eeKinematics_(eeKinematics.clone()),
      userParam_(paramFile) {
  const auto& model = pinocchioInterfaceMeasured_.getModel();
  numDecisionVars_ = model.nv + 3 * switched_model::NUM_CONTACT_POINTS; // \dot v, f
  qMeasured_ = vector_t(model.nq);
  vMeasured_ = vector_t(model.nv);
  qDesired_ = vector_t(model.nq);
  vDesired_ = vector_t(model.nv);
  inputLast_ = vector_t::Zero(switched_model::INPUT_DIM);
  baseAccDesired_ = vector_t(6);

  // armEeFrameIdx_ = model.getBodyId(armEeKinematics_->getIds()[0]);

  base_j_ = matrix_t(6, model.nv);
  base_dj_ = matrix_t(6, model.nv);

  legTorqueLimits_ = vector3_t::Zero();
  // legTorqueLimits_ << 126.1, 111.4, 195.8;
  legTorqueLimits_ = userParam_.TauMax;
  frictionCoeff_ = userParam_.mu;

  Kp_body_ = userParam_.Kp_body;
  Kd_body_ = userParam_.Kd_body;

  Kp_ori_ = userParam_.Kp_ori;
  Kd_ori_ = userParam_.Kd_ori;

  Kp_swing_ = userParam_.Kp_foot;
  Kd_swing_ = userParam_.Kd_foot;

  for (const auto& bodyName : endEffectorIds) {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
    std::cout << "bodyName: " << bodyName << "\t id: " << endEffectorFrameIds_.back() << "\n";
  }

  // ros::NodeHandle nh;
  // endEfferotMeasuredVelPub_ = nh.advertise<std_msgs::Float32MultiArray>("wbc/eeMeasuredVel", 1);
}

void QuadrupedWbc::updateMeasured(const ocs2::vector_t &rbdStateMeasured) {
  qMeasured_.setZero();
  vMeasured_.setZero();

  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  // qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  const vector3_t eulerXYZ = rbdStateMeasured.head<3>();
  qMeasured_.segment<4>(3) = quaternionBaseToOrigin(eulerXYZ).coeffs();
  joint_coordinate_t q_j = rbdStateMeasured.segment(6, JOINT_COORDINATE_SIZE);
  qMeasured_.tail(JOINT_COORDINATE_SIZE) = mapping_.getPinocchioJointVector(q_j);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE + 3);
  vMeasured_.segment<3>(3) = rbdStateMeasured.segment<3>(BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE );
  joint_coordinate_t dq_j = rbdStateMeasured.segment(BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE + 6, JOINT_COORDINATE_SIZE);
  vMeasured_.tail(JOINT_COORDINATE_SIZE) = mapping_.getPinocchioJointVector(dq_j);


  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();


  // For floating base EoM task
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * NUM_CONTACT_POINTS, model.nv);
  for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, model.nv);
      pinocchio::getFrameJacobian(model, data, endEffectorFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      j_.block(3 * i, 0, 3, model.nv) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * NUM_CONTACT_POINTS, model.nv);
  for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, model.nv);
      pinocchio::getFrameJacobianTimeVariation(model, data, endEffectorFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      dj_.block(3 * i, 0, 3, model.nv) = jac.template topRows<3>();
  }

  // For base motion tracking task
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> base_j, base_dj;
  base_j.setZero(6, model.nv);
  base_dj.setZero(6, model.nv);

  // std::cout << "base id: " << model.getBodyId("base") << "\n";
  pinocchio::getFrameJacobian(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_j);
  pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_dj);
  base_j_.setZero(); base_j_ = base_j;
  base_dj_.setZero(); base_dj_ = base_dj;
}

void QuadrupedWbc::updateDesired(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired, ocs2::scalar_t period) {
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  qDesired_.setZero();
  vDesired_.setZero();

  qDesired_.head<3>() = stateDesired.segment<3>(3);
  // qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  const vector3_t eulerXYZ = stateDesired.head<3>();
  qDesired_.segment<4>(3) = quaternionBaseToOrigin(eulerXYZ).coeffs();
  joint_coordinate_t q_j = stateDesired.tail(JOINT_COORDINATE_SIZE);
  qDesired_.tail(JOINT_COORDINATE_SIZE) =  mapping_.getPinocchioJointVector(q_j);

  // updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired_);

  vDesired_.head<3>() = stateDesired.segment<3>(BASE_COORDINATE_SIZE + 3);
  vDesired_.segment<3>(3) = stateDesired.segment<3>(BASE_COORDINATE_SIZE);
  joint_coordinate_t dq_j = inputDesired.segment<JOINT_COORDINATE_SIZE>(3 * NUM_CONTACT_POINTS);
  vDesired_.tail(JOINT_COORDINATE_SIZE) = mapping_.getPinocchioJointVector(dq_j);

  pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);
  pinocchio::computeJointJacobians(model, data, qDesired_);
  pinocchio::updateFramePlacements(model, data);

  // update base acc desired
  joint_coordinate_t ddq_j = (inputDesired - inputLast_).tail(JOINT_COORDINATE_SIZE) / period;
  vector6_t ddq_base = (vDesired_.head<6>() - lastBaseTwist_) / period;
  const vector3_t linear_acc = ddq_base.head<3>();
  ddq_base.head(3) = switched_model::rotateVectorBaseToOrigin(linear_acc, eulerXYZ);
  const vector3_t angluar_acc = ddq_base.tail<3>();
  ddq_base.tail(3) = switched_model::rotateVectorBaseToOrigin(angluar_acc, eulerXYZ);
  jointAccel_ = mapping_.getPinocchioJointVector(ddq_j);
  inputLast_ = inputDesired;
  lastBaseTwist_ = vDesired_.head<6>();

  baseAccDesired_ = ddq_base;
  // std::cout << "baseAccDesired_: " << baseAccDesired_.transpose() << "\n";
  // const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  // const Matrix6 Ab = A.template leftCols<6>();
  // const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  // auto Aj = A.rightCols(info_.actuatedDofNum);
  // const auto ADot = pinocchio::dccrba(model, data, qDesired_, vDesired_);
  // Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  // centroidalMomentumRate.noalias() -= ADot * vDesired_;
  // centroidalMomentumRate.noalias() -= Aj * jointAccel_;

  // baseAccDesired_.setZero();
  // baseAccDesired_ = AbInv * centroidalMomentumRate;
}

vector_t QuadrupedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                              const vector_t& rbdStateMeasured, size_t mode, scalar_t period, scalar_t time) {
  updateMode(mode);
  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired, period);

  // if(time - last_time_ > ros::Duration(0.01).toSec())
  // {
  //     publishMsg();
  //     last_time_ = time;
  // }
  last_time_ = time;

  return {};
}

void QuadrupedWbc::updateMode(size_t mode) {
  contactFlag_ = switched_model::modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
        numContacts_++;
    }
  }
}

vector_t QuadrupedWbc::updateCmd(ocs2::vector_t x_optimal) {
  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t Mj, Jj_T, Jj;
  vector_t hj;
  Mj = data.M.bottomRows(JOINT_COORDINATE_SIZE);
  Jj = j_; Jj.middleRows(3, 3) = j_.middleRows(6, 3); Jj.middleRows(6, 3) = j_.middleRows(3, 3);
  Jj_T = Jj.transpose().bottomRows(JOINT_COORDINATE_SIZE);//j_.transpose().bottomRows(JOINT_COORDINATE_SIZE);
  hj = data.nle.bottomRows(JOINT_COORDINATE_SIZE);
  matrix_t a = (matrix_t(JOINT_COORDINATE_SIZE, getNumDecisionVars())<< Mj, -Jj_T).finished();

  vector_t forceInBaseFrame =  x_optimal.tail<3 * NUM_CONTACT_POINTS>();
  vector_t forceInWorldFrame = vector_t::Zero(3 * NUM_CONTACT_POINTS);
  Eigen::Quaterniond quatMeasured(qMeasured_.segment<4>(3));
  vector3_t eulerXYZ = quatMeasured.toRotationMatrix().eulerAngles(0, 1, 2);
  for(size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
    vector3_t force = forceInBaseFrame.segment<3>(3 * i);
    forceInWorldFrame.segment<3>(3 * i) = switched_model::rotateVectorBaseToOrigin(force, eulerXYZ);
  }

  x_optimal.tail(3 * NUM_CONTACT_POINTS) = forceInWorldFrame;
  vector_t torque_optimal = a * x_optimal + hj; // [LF LH RF RH]
  vector_t torque_optimal_ocs2Order(JOINT_COORDINATE_SIZE);
  torque_optimal_ocs2Order << torque_optimal.head<3>(), torque_optimal.segment<3>(6), 
                              torque_optimal.segment<3>(3), torque_optimal.tail<3>(); // [LF RF LH RH]
  vector_t cmd = (vector_t(numDecisionVars_ + JOINT_COORDINATE_SIZE) << x_optimal, torque_optimal_ocs2Order).finished();

  return cmd;
}
// EoM
// [Mb, -J^Tb]x = -hb
Task QuadrupedWbc::formulateFloatingBaseEomTask() {
  matrix_t a(6, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();

  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t Mb, Jb_T;
  vector_t hb;
  Mb = data.M.topRows(6);
  hb = data.nle.topRows(6);
  Jb_T = j_.transpose().topRows(6);

  std::cout << "Mb: \n" << Mb << "\n";

  a << Mb, -Jb_T;
  b = -hb;
  return {a, b, matrix_t(), matrix_t()};
}

// torque limit
// tau_min - hj <= [Mj, -Jj^T]x <= tau_max - hj
Task QuadrupedWbc::formulateTorqueLimitsTask() {
  matrix_t d(2 * JOINT_COORDINATE_SIZE, numDecisionVars_);
  vector_t f(d.rows());
  d.setZero();
  f.setZero();

  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t Mj, Jj_T;
  vector_t hj;
  Mj = data.M.bottomRows(JOINT_COORDINATE_SIZE);
  Jj_T = j_.transpose().bottomRows(JOINT_COORDINATE_SIZE);
  hj = data.nle.bottomRows(JOINT_COORDINATE_SIZE);

  d.block(0, 0, JOINT_COORDINATE_SIZE, numDecisionVars_) << Mj, -Jj_T;
  d.block(JOINT_COORDINATE_SIZE, 0, JOINT_COORDINATE_SIZE, numDecisionVars_) << -Mj, Jj_T;

  f << legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, legTorqueLimits_,
          legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, legTorqueLimits_;
  f.head(JOINT_COORDINATE_SIZE) -= hj;
  f.tail(JOINT_COORDINATE_SIZE) += hj;

  return {matrix_t(), vector_t(), d, f};
}

Task QuadrupedWbc::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, vMeasured_.rows()) = j_.block(3 * i, 0, 3, vMeasured_.rows());
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, vMeasured_.rows()) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task QuadrupedWbc::formulateFrictionConeTask() {
  matrix_t a(3 * (NUM_CONTACT_POINTS - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
          1, 0, -frictionCoeff_,
          -1, 0, -frictionCoeff_,
          0, 1, -frictionCoeff_,
          0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (NUM_CONTACT_POINTS - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

Task QuadrupedWbc::formulateBaseAccelTask() {
  matrix_t a(3, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 3, 3) = matrix_t::Identity(3, 3);

  vector3_t b = baseAccDesired_;

  return {a, b, matrix_t(), vector_t()};
}

Task QuadrupedWbc::formulateBaseAngularMotionTask(){
  matrix_t a(3, numDecisionVars_);
  vector_t b(a.rows());

  a.setZero();
  b.setZero();

  a.block(0, 0, 3, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE) = base_j_.block(3, 0, 3, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE);
  a.middleCols(3, 3) = matrix_t::Identity(3, 3);
  std::cout << "[formulateBaseAngularMotionTask] a: \n" << a << "\n";
  std::cout << "base_j: \n" << base_j_ << "\n";

  Eigen::Quaterniond quatMeasured(qMeasured_.segment<4>(3));
  vector3_t eulerXYZ = eulerAnglesFromQuaternionBaseToOrigin(quatMeasured);

  // from derivative euler to angular
  vector3_t omegaMeasuredInBase = vMeasured_.segment<3>(3);
  vector3_t vMeasuredGlobal = omegaMeasuredInBase;//rotateVectorBaseToOrigin(omegaMeasuredInBase, eulerXYZ);
          // getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
  vector3_t omegaDesiredInBase = vDesired_.segment<3>(3);
  vector3_t vDesiredGlobal =  omegaDesiredInBase;//rotateVectorBaseToOrigin(omegaDesiredInBase, eulerXYZ);
          // getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vDesired_.segment<3>(3));

  // from euler to rotation
  Eigen::Quaterniond quatDesired(qDesired_.segment<4>(3));
  vector3_t eulerAnglesDesired = eulerAnglesFromQuaternionBaseToOrigin(quatMeasured);

  matrix3_t rotationBaseMeasuredToWorld = quatMeasured.toRotationMatrix();
          // getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
  matrix3_t rotationBaseReferenceToWorld = quatDesired.toRotationMatrix();
          // getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

  vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);
  error.setZero();

  // desired acc
  const vector3_t base_angular_acc = baseAccDesired_.head(3);
  vector3_t accDesired = base_angular_acc;//rotateVectorBaseToOrigin(base_angular_acc, eulerXYZ);
  accDesired.setZero();
  vDesiredGlobal.setZero();
  vMeasuredGlobal.setZero();
  // vector3_t accDesired = rotateVectorBaseToOrigin(base_angular_acc, eulerXYZ);
    // getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vDesired_.segment<3>(3), baseAccDesired_.segment<3>(3));

  b = accDesired + baseAngularKp_  * error + baseAngularKd_ * (vDesiredGlobal - vMeasuredGlobal)
                - base_dj_.block(3, 0, 3, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE) * vMeasured_;

  return {matrix_t(), vector_t(), matrix_t(), vector_t()};
}

// [J, 0] x = \dot V - \dotJ v // active only in swing phase -> always active
Task QuadrupedWbc::formulateSwingLegTask() {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (NUM_CONTACT_POINTS - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();

  matrix3_t Kp = matrix3_t::Identity(3, 3);
  matrix3_t Kd = matrix3_t::Identity(3, 3);
  Kp.diagonal() = Kp_swing_;
  Kd.diagonal() = Kd_swing_;
  size_t j = 0;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
    if (!contactFlag_[i]) {// [LF, RF, LH, RH]
      vector3_t accel = Kp * (posDesired[i] - posMeasured[i]) + Kd * (velDesired[i] - velMeasured[i]);
      //Lu Chen: //TODO
      // (on z-axis) Kf * (F_d - F_current) + Kp * (posDesired[i] - posMeasured[i]) + Kd * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE) = j_.block(3 * i, 0, 3, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

// [0, I] x = GRFs
// inputDesired [LF, RF, LH, RH]
Task QuadrupedWbc::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * NUM_CONTACT_POINTS, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();

  for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) {
      a.block(3 * i, BASE_COORDINATE_SIZE+JOINT_COORDINATE_SIZE+ 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  // b = inputDesired.head(a.rows());
  // TODO
  b << inputDesired.head<3>(), inputDesired.segment<3>(6), inputDesired.segment<3>(3), inputDesired.segment<3>(9);

  return {a, b, matrix_t(), vector_t()};
}

void QuadrupedWbc::loadTasksSetting(const std::string &taskFile, bool verbose) {
  // Load task file
  legTorqueLimits_ = vector_t(3);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", legTorqueLimits_);

  // arm torque limit
  // armTorqueLimits_ = vector_t(6);
  // armTorqueLimits_ = pinocchioInterfaceMeasured_.getModel().effortLimit.tail(6);

  // if (verbose) {
  //     std::cerr << "\n #### Torque Limits Task:";
  //     std::cerr << "\n #### =============================================================================\n";
  //     std::cerr << "\n #### HAA HFE KFE: " << legTorqueLimits_.transpose() << "\n";
  //     std::cerr << " #### =============================================================================\n";
  //     std::cerr << "\n #### manipulator joint: " << armTorqueLimits_.transpose() << "\n";
  //     std::cerr << " #### =============================================================================\n";
  // }

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
      std::cerr << "\n #### Friction Cone Task:";
      std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
      std::cerr << " #### =============================================================================\n";
  }

  prefix = "swingLegTask.";
  if (verbose) {
      std::cerr << "\n #### Swing Leg Task:";
      std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);

}

void QuadrupedWbc::modifyWbcParameters() {
    frictionCoeff_ = userParam_.mu;

    Kp_body_ = userParam_.Kp_body;
    Kd_body_ = userParam_.Kd_body;

    Kp_ori_ = userParam_.Kp_ori;
    Kd_ori_ = userParam_.Kd_ori;

    Kp_swing_ = userParam_.Kp_foot;
    Kd_swing_ = userParam_.Kd_foot;
}

}
}