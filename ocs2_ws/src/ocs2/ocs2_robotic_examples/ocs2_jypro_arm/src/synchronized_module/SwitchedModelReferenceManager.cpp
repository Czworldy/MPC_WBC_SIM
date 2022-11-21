#include "SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
        : ReferenceManager(TargetTrajectories(), ModeSchedule()),
          gaitSchedulePtr_(std::move(gaitSchedulePtr)), 
          swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
    return modeNumber2StanceLimb(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, 
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
    const auto timeHorizon = finalTime - initTime;
    modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

    const scalar_t terrainHeight = 0.03; // For JYPro
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}



} // namespace legged_robot
} // namespace ocs2