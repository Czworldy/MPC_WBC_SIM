#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "SwingTrajectoryPlanner.h"
#include "GaitSchedule.h"
#include "MotionPhaseDefinition.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
    public:
        SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);
        
        ~SwitchedModelReferenceManager() override = default;

        contact_flag_t getContactFlags(scalar_t time) const;

        const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

        const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

    private:
        void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                              ModeSchedule& modeSchedule) override;
        
        std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
        std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

} // namespace legged_robot
} // namespace ocs2