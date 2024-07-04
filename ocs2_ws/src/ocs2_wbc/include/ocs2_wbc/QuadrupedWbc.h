/*
 * @Author: Jiyu Yu 
 * @Date: 2024-06-27 18:50:37 
 * @Last Modified by: Jiyu Yu
 * @Last Modified time: 2024-07-02 10:14:22
 */

#pragma once
#include "ocs2_wbc/Task.h"
#include <ocs2_jypro/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include "ocs2_wbc/UserParameter.h"
#include "ocs2_wbc/LegLogic.h"

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_anymal_models/QuadrupedPinocchioMapping.h>


namespace ocs2 {
namespace wbc{

class QuadrupedWbc {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;

  using contact_flag_t = std::array<bool, 4>;
  QuadrupedWbc(const ocs2::PinocchioInterface &pinocchioInterface, 
               const anymal::QuadrupedPinocchioMapping& quadrupedPinocchioMapping,
               const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
               std::vector<std::string> endEffectorIds,
               const std::string& paramFile);
  ~QuadrupedWbc() = default;
  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                          const vector_t& rbdStateMeasured, size_t mode, scalar_t period, scalar_t time);
  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);
  virtual void modifyWbcParameters();
  UserParameter& getUserParam() { return userParam_; }
 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, ocs2::scalar_t period);
  void updateMode(size_t mode);
  vector_t updateCmd(vector_t x_optimal);
  size_t getNumDecisionVars() const { return numDecisionVars_; }
    

  Task formulateFloatingBaseEomTask();
  Task formulateNoContactMotionTask();
  Task formulateTorqueLimitsTask();
  Task formulateFrictionConeTask();

  Task formulateBaseAccelTask();
  Task formulateBaseAngularMotionTask();
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;
 private:
  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  const anymal::QuadrupedPinocchioMapping& mapping_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  vector_t qMeasured_, vMeasured_, inputLast_;
  vector_t qDesired_, vDesired_, baseAccDesired_;
  vector_t jointAccel_;
  matrix_t j_, dj_;
  // matrix_t arm_j_, arm_dj_;
  matrix_t base_j_, base_dj_;

  // Task Parameters:
  vector_t legTorqueLimits_;
  scalar_t frictionCoeff_ = 0.5, swingKp_ = 350., swingKd_ = 37.;

  vector3_t Kp_swing_, Kd_swing_;
  vector3_t Kp_body_, Kd_body_;
  vector3_t Kp_ori_, Kd_ori_;

  scalar_t baseHeightKp_ = 400., baseHeightKd_ = 40.;
  scalar_t baseAngularKp_ = 400, baseAngularKd_ = 40;

  vector6_t lastBaseTwist_ = vector6_t::Zero();


  // User param
  UserParameter userParam_;
  std::vector<size_t> endEffectorFrameIds_;
  // ros::Publisher endEfferotMeasuredVelPub_;
  scalar_t last_time_;
};

} // namespace wbc
} // namespace ocs2
