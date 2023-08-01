#pragma once

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>



namespace ocs2 {
namespace legged_robot {

class LeggedRobotEndEffectorCost final : public StateInputCost {
 public:
  LeggedRobotEndEffectorCost(matrix_t Q, matrix_t R, const SwitchedModelReferenceManager& referenceManager,
                            const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                            size_t contactPointIndex);

  ~LeggedRobotEndEffectorCost() override = default;
  LeggedRobotEndEffectorCost* clone() const override;

  bool isActive(scalar_t time) const override;

  /** Cost evaluation */
  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComputation) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComputation) const override;

 private:
  LeggedRobotEndEffectorCost(const LeggedRobotEndEffectorCost& rhs) = default;

  // const PinocchioEndEffectorKinematicsCppAd& endEffectorKinematics_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  PinocchioEndEffectorKinematicsCppAd* endEffectorKinematics_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  matrix_t Q_, R_;
  size_t contactPointIndex_;

  std::function<void(const ad_vector_t&, ad_vector_t&)> positionFunc_;
  std::function<void(const ad_vector_t&, ad_vector_t&)> velocityFunc_;

};

}  // namespace legged_robot
}  // namespace ocs2
