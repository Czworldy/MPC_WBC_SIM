/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <ocs2_jypro/LeggedRobotPreComputation.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner, 
                                                     const FootPlacementPlanner& footPlacementPlanner,
                                                     std::unique_ptr<legged::LeggedIKSolver> leggedIKSolverPtr,
                                                     ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      footPlacnementPlannerPtr_(&footPlacementPlanner),
      leggedIKSolverPtr_(std::move(leggedIKSolverPtr)),
      settings_(std::move(settings)) {
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  swingTimeLeft_.resize(info_.numThreeDofContacts);
  footPlacementConstraints_.resize(info_.numThreeDofContacts);
  eeReference_.resize(info_.numThreeDofContacts);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation* LeggedRobotPreComputation::clone() const {
  return new LeggedRobotPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
    }
    return config;
  };

  // lambda to get left swing time for foot placement constraints
  auto swingTimeLeftLambda = [&](size_t footIndex) {
    return swingTrajectoryPlannerPtr_->getSwingTimeLeft(footIndex, t);
  };

  auto footPlacementPoint = [&](size_t footIndex) {
    vector3_t point = footPlacnementPlannerPtr_->getFootPlacementConstraint(footIndex, t);
    // std::cout << "foot index:" << footIndex << "\t" << point.transpose() << std::endl;
    scalar_t tol = 0.05;

    Eigen::Matrix<scalar_t, 6, 1> constraint, b;
     b  << -point[0], point[0], -point[1], point[1], -point[2], point[2];
    constraint = b.array() + tol;
    
    return constraint;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      swingTimeLeft_[i] = swingTimeLeftLambda(i);
      footPlacementConstraints_[i] = footPlacementPoint(i);
      // std::cout << "preCompute times: " << t << "\t" << footPlacementConstraints_[i].transpose() << std::endl;

    }
    // Eigen::Map<Eigen::Matrix<scalar_t, 4, 1>> times(swingTimeLeft_.data());
    // std::cout << "preCompute times: " << footPlacementConstraints_.transpose() << std::endl;
  }

  auto eeReferece = [&](size_t footIndex) {
    vector_t reference(6);
    const scalar_t xPosition = swingTrajectoryPlannerPtr_->getXpositionConstraint(footIndex, t);
    const scalar_t yPosition = swingTrajectoryPlannerPtr_->getYpositionConstraint(footIndex, t);
    const scalar_t zPosition = swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);

    const scalar_t xVelocity = swingTrajectoryPlannerPtr_->getXvelocityConstraint(footIndex, t);
    const scalar_t yVelocity = swingTrajectoryPlannerPtr_->getYvelocityConstraint(footIndex, t);
    const scalar_t zVelocity = swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t);
    reference << xPosition, yPosition, zPosition, xVelocity, yVelocity, zVelocity;
    // std::cout << "ref: " <<  reference.transpose() << std::endl;
    return reference;
  };

  // auto eeIKSolver = [&](size_t footIndex, const vector3_t& pos) {

  //   leggedIKSolverPtr_->setBasePos(x.segment<6>(6));
  //   vector3_t res = leggedIKSolverPtr_->solveIK(pos, footIndex);
  //   std::cout << "res: " <<  res.transpose() << std::endl;
  // };

  if (request.contains(Request::Cost)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeReference_[i] = eeReferece(i);
      // eeIKSolver(i, eeReference_[i].segment<3>(0));
    }
    
  }
}

}  // namespace legged_robot
}  // namespace ocs2
