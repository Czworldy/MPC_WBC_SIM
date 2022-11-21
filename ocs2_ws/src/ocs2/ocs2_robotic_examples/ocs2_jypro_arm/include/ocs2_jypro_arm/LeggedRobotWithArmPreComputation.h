#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "ModelSettings.h"
#include "EndEffectorLinearConstraint.h"
#include "SwingTrajectoryPlanner.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class LeggedRobotWithArmPreComputation : public PreComputation {
    public:
        LeggedRobotWithArmPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, 
                                         const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
        ~LeggedRobotWithArmPreComputation() override = default;

        LeggedRobotWithArmPreComputation* clone() const override;

        void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

        const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

        PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
        const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

    private:
        LeggedRobotWithArmPreComputation(const LeggedRobotWithArmPreComputation& other) = default;

        PinocchioInterface pinocchioInterface_;
        CentroidalModelInfo info_;
        const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
        const ModelSettings settings_;

        std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
};

} // namespace legged_robot
} // namespace ocs2