#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ros/ros.h>

#include "GaitSchedule.h"
#include "ModeSequenceTemplate.h"
#include "MotionPhaseDefinition.h"

namespace ocs2 {
namespace legged_robot {

class GaitReceiver : public SolverSynchronizedModule {
    public:
        GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string& robotName);

        void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;
        void postSolverRun(const PrimalSolution& primalSolution) override{};

    private:
        void mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg);

        std::shared_ptr<GaitSchedule> gaitSchedulePtr_;

        ros::Subscriber mpcModeSequenceSubscriber_;

        std::mutex receivedGaitMutex_;
        std::atomic_bool gaitUpdated_;
        ModeSequenceTemplate receivedGait_;
};

} // namespace legged_robot
} // namespace ocs2