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

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "ocs2_jypro/common/ModelSettings.h"
#include "ocs2_jypro/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_jypro/foot_planner/SwingTrajectoryPlanner.h"
// #include "ocs2_jypro/foot_planner/FootPlacementPlanner.h"
#include "ocs2_jypro/foot_planner/FootConstraintsPlanner.h"
#include "ocs2_jypro/foot_planner/LeggedIKSolver.h"

#include "ocs2_jypro/synchronized_module/TerrainReceiver.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class LeggedRobotPreComputation : public PreComputation {
 public:
  LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner, 
                            // const FootPlacementPlanner& fottPlacementPlanner,
                            const FootConstraintsPlanner& footConstraintsPlanner,
                            std::unique_ptr<legged::LeggedIKSolver> leggedIKSolverPtr,
                            std::shared_ptr<TerrainEstData> terrainEstDataPtr,
                            ModelSettings settings);
  ~LeggedRobotPreComputation() override = default;

  LeggedRobotPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  const std::vector<scalar_t>& getSwingTimeLeft() const { return swingTimeLeft_; }
  // const std::vector<Eigen::Matrix<scalar_t, 6, 1>>& getFootPlacementConstraint() const { return footPlacementConstraints_; }
  const std::vector<FootConstraints>& getFootPlacementConstraint() const { return footPlacementConstraints_; }
  const std::vector<vector_t> getEEReference() const { return eeReference_; }

  const std::shared_ptr<TerrainEstData>& getTerrainEstDataPtr() const { return terrainEstDataPtr_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  LeggedRobotPreComputation(const LeggedRobotPreComputation& other) = default;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  // const FootPlacementPlanner* footPlacnementPlannerPtr_;
  const FootConstraintsPlanner* footConstraintsPlannerPtr_;
  const ModelSettings settings_;

  std::shared_ptr<legged::LeggedIKSolver> leggedIKSolverPtr_;

  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
  std::vector<scalar_t> swingTimeLeft_;
  // std::vector<Eigen::Matrix<scalar_t, 6, 1>> footPlacementConstraints_;
  std::vector<FootConstraints> footPlacementConstraints_;

  std::vector<vector_t> eeReference_;
  std::shared_ptr<TerrainEstData> terrainEstDataPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
