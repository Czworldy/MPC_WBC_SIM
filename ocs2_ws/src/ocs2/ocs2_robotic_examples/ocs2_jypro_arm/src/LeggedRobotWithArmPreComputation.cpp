#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <LeggedRobotWithArmPreComputation.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmPreComputation::LeggedRobotWithArmPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, 
                                                                   const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
          settings_(std::move(settings)) {
    eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmPreComputation* LeggedRobotWithArmPreComputation::clone() const {
    return new LeggedRobotWithArmPreComputation(* this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
    if(!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
        return;
    }

    // lambda to set config for normal velocity constraints
    auto eeNormalVelConConfig = [&](size_t footIndex) {
        EndEffectorLinearConstraint::Config config;
        config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
        if(!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
            config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
            config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
        }
        return config;
    };

    if(request.contains(Request::Constraint)) {
        for (size_t i = 0; i < 4; i++) { // for legs
            eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
        }
    }
}

} // namespace legged_robot
} // namespace ocs2