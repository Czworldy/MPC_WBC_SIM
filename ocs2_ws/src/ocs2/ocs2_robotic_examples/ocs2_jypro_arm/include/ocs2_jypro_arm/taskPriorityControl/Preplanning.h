#pragma once


#include <pinocchio/fwd.hpp>
#include <prePlanDefinition.h>
#include <TaskPriorityControl.h>
#include <PinocchioInterfaceForArm.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

namespace ocs2 {
namespace legged_robot {
namespace arm {

#define LENGTH 2

class PrePlanning final {

    public:
        PrePlanning();
        ~PrePlanning(){};

        void eelineTrajectoryPlanning(const SystemObservation& observation, 
                                      const vector_t& defaultLegJointState,
                                      TargetTrajectories& trajectory);
        void setUpMotion(const SystemObservation& observation, 
                         const GripperBasePosition& targetPosition, 
                         const scalar_t& targetTime);
        void keepMotion(const SystemObservation& observation, 
                        const vector_t& defaultLegJointState, 
                        TargetTrajectories& trajectory);

    private:
        void setUpInitialStates(const SystemObservation& observation);
        void setUpGoalStates(const GripperBasePosition& targetPosition);
        void setUpGoalTime(const scalar_t& targetTime);
        void setUpPlanTime(const scalar_t& planTime);
        void cubicSplinePlan(const SystemObservation& observation);
        vector6_t getGripperPosVel(const SystemObservation& observation);
        vector3_t cubicTrajectory_d(float startPoint, float finalPoint, float finalTime, float time_traj);
        bool isMotionFinished();

        GripperStatesForPlan initialGripperStates_, goalGripperStates_, planedGripperStates_;
        BaseStatesForPlan initialBaseStates_, goalBaseStates_, planedBaseStates_;
        scalar_t timeGoal_, timePlan_, timeNow_;
        long long int iterator_;
        std::unique_ptr<PinocchioInterfaceForArm> pinocchioInterfacePtr_;
        std::unique_ptr<TaskPriorityControl> taskPriorityControlPtr_;
        ArmSettings armSettings_;
        vector_t jointSpacePosition_;
        vector_t jointSpacePositionTmp_;
        vector_t jointSpaceVelocity_;
        GripperBaseVelocity endEffectorVelocity_;
        int dimStates;
        scalar_t planHorizon_;
        bool isPrePlanSetUp_;
        scalar_t planTimeInterval_;
        scalar_t loopTime_;
        vector_t stateTrajectoryLast_, inputTrajectoryLast_;
        vector_t Kp_plan_;
};

} // namespace arm
}
} // namespace ocs2
