#include <Preplanning.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace legged_robot {
namespace arm {

PrePlanning::PrePlanning() {
    // URDF Model -> Pinocchio Model
    std::string urdfPath = "/home/dqwang/MPC_WBC_arm/ocs2_ws/src/X20/urdf/ARM_ocs2.urdf";
    pinocchioInterfacePtr_.reset(new PinocchioInterfaceForArm(urdf::parseURDFFile(urdfPath), armSettings_.jointNames));
    taskPriorityControlPtr_.reset(new TaskPriorityControl());
    jointSpacePosition_.resize(pinocchioInterfacePtr_->nq);
    jointSpacePositionTmp_.resize(pinocchioInterfacePtr_->nq);
    jointSpaceVelocity_.resize(pinocchioInterfacePtr_->nq);
    planHorizon_ = 0.5;
    planTimeInterval_ = planHorizon_ / (LENGTH - 1);
    loopTime_ = 0.02;
}

void PrePlanning::setUpMotion(const SystemObservation& observation, 
                 const GripperBasePosition& targetPosition, 
                 const scalar_t& targetTime) {
    setUpInitialStates(observation);
    setUpGoalStates(targetPosition);
    setUpGoalTime(targetTime);

    iterator_ = 0;
    timeNow_ = 0;
}

void PrePlanning::eelineTrajectoryPlanning(const SystemObservation& observation, 
                                           const vector_t& defaultLegJointState,
                                           TargetTrajectories& trajectory) {

    if(!isMotionFinished()) {
        timeNow_ = observation.time;

        trajectory.timeTrajectory.resize(LENGTH);
        trajectory.stateTrajectory.resize(LENGTH);
        trajectory.inputTrajectory.resize(LENGTH);
        // iteration 0
        int dimStates(observation.state.size());
        
        for (int i(0); i < 6; i++) {
            jointSpacePosition_[i] = observation.state[6+i];
            jointSpacePosition_[i+6] = observation.state[dimStates-6+i];
        }

        setUpPlanTime(observation.time);
        cubicSplinePlan();

        taskPriorityControlPtr_->TPcontrolLaw(jointSpacePosition_,
                                              endEffectorVelocity_,
                                              jointSpaceVelocity_);

        trajectory.timeTrajectory[0] = observation.time;
        trajectory.stateTrajectory[0].resize(observation.state.size());
        trajectory.stateTrajectory[0] << vector_t::Zero(6), jointSpacePosition_.head(6), defaultLegJointState, jointSpacePosition_.tail(6);
        trajectory.inputTrajectory[0] = vector_t::Zero(observation.input.size());

        jointSpacePositionTmp_ = jointSpacePosition_;
        jointSpacePosition_ = jointSpacePositionTmp_ + jointSpaceVelocity_ * planTimeInterval_;

        // std::cout << "[PrePlanning::eelineTrajectoryPlanning] jointSpaceVelocity_: "     << jointSpaceVelocity_ << std::endl;
        // std::cout << "[PrePlanning::eelineTrajectoryPlanning] jointSpacePosition_: "     << jointSpacePosition_ << std::endl;

        // iteration 1->
        for (int i(0); i < (LENGTH - 1); i++) {
            scalar_t planTime = observation.time + planTimeInterval_ * (i + 1); 
            setUpPlanTime(planTime);
            cubicSplinePlan();

            taskPriorityControlPtr_->TPcontrolLaw(jointSpacePosition_,
                                                endEffectorVelocity_,
                                                jointSpaceVelocity_);

            trajectory.timeTrajectory[i+1] = planTime;
            trajectory.stateTrajectory[i+1].resize(observation.state.size());
            trajectory.stateTrajectory[i+1] << vector_t::Zero(6), jointSpacePosition_.head(6), defaultLegJointState, jointSpacePosition_.tail(6);
            trajectory.inputTrajectory[i+1] = vector_t::Zero(observation.input.size());

            jointSpacePositionTmp_ = jointSpacePosition_;
            jointSpacePosition_ = jointSpacePositionTmp_ + jointSpaceVelocity_ * planTimeInterval_;

            // std::cout << "[PrePlanning::eelineTrajectoryPlanning] jointSpaceVelocity_: "     << jointSpaceVelocity_ << std::endl;
            // std::cout << "[PrePlanning::eelineTrajectoryPlanning] jointSpacePosition_: "     << jointSpacePosition_ << std::endl;
        }

        stateTrajectoryLast_ = trajectory.stateTrajectory[0];
        inputTrajectoryLast_ = trajectory.inputTrajectory[0];
    }
    else {
        keepMotion(observation, defaultLegJointState, trajectory);
    }


    // std::cout << "[PrePlanning::eelineTrajectoryPlanning] trajectory.timeTrajectory \n" << trajectory.timeTrajectory[0] << "\n" << trajectory.timeTrajectory[1] << std::endl;
    // std::cout << "[PrePlanning::eelineTrajectoryPlanning] trajectory.stateTrajectory \n" << trajectory.stateTrajectory[0] << "\n" << trajectory.stateTrajectory[1] << std::endl;
}

void PrePlanning::setUpInitialStates(const SystemObservation& observation) {
    // initial base states
    initialBaseStates_.x[0]     = observation.state[6];
    initialBaseStates_.y[0]     = observation.state[7];
    initialBaseStates_.z[0]     = observation.state[8];
    initialBaseStates_.yaw[0]   = observation.state[9];
    initialBaseStates_.pitch[0] = observation.state[10];
    initialBaseStates_.roll[0]  = observation.state[11];

    // initial gripper states
    vector_t initialJointPositions(pinocchioInterfacePtr_->nq);
    initialJointPositions[0] = initialBaseStates_.x[0];
    initialJointPositions[1] = initialBaseStates_.y[0];
    initialJointPositions[2] = initialBaseStates_.z[0];
    initialJointPositions[3] = initialBaseStates_.yaw[0];
    initialJointPositions[4] = initialBaseStates_.pitch[0];
    initialJointPositions[5] = initialBaseStates_.roll[0];
    int dimState = observation.state.size();
    for (int i(0); i < 6; i++) {
        initialJointPositions[6 + i] = observation.state[dimState - 6 + i];
    }

    vector_t initialJointVelocities(pinocchioInterfacePtr_->nq);
    // initialJointVelocities[0] = observation.baseVelocityForX20[0];
    // initialJointVelocities[1] = observation.baseVelocityForX20[1];
    // initialJointVelocities[2] = observation.baseVelocityForX20[2];
    // initialJointVelocities[3] = observation.baseVelocityForX20[3];
    // initialJointVelocities[4] = observation.baseVelocityForX20[4];
    // initialJointVelocities[5] = observation.baseVelocityForX20[5]; // TODO: Some problems here
    // int dimInput = observation.input.size();
    // for (int i(0); i < 6; i++) {
    //     initialJointVelocities[6 + i] = observation.input[dimInput - 6 + i];
    // }

    vector6_t gripperPos, gripperVel;
    pinocchioInterfacePtr_->gripperPosVel(initialJointPositions, initialJointVelocities, gripperPos, gripperVel);

    initialGripperStates_.x[0]     = gripperPos[0];
    initialGripperStates_.y[0]     = gripperPos[1];
    initialGripperStates_.z[0]     = gripperPos[2];
    initialGripperStates_.yaw[0]   = gripperPos[3];
    initialGripperStates_.pitch[0] = gripperPos[4];
    initialGripperStates_.roll[0]  = gripperPos[5];

    // std::cout << "[PrePlanning::setUpInitialStates] initialGripperStates_.x: "     << initialGripperStates_.x[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialGripperStates_.y: "     << initialGripperStates_.y[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialGripperStates_.z: "     << initialGripperStates_.z[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialGripperStates_.yaw: "   << initialGripperStates_.yaw[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialGripperStates_.pitch: " << initialGripperStates_.pitch[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialGripperStates_.roll: "  << initialGripperStates_.roll[0] << std::endl;

    // std::cout << "[PrePlanning::setUpInitialStates] initialBaseStates_.x: "     << initialBaseStates_.x[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialBaseStates_.y: "     << initialBaseStates_.y[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialBaseStates_.z: "     << initialBaseStates_.z[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialBaseStates_.yaw: "   << initialBaseStates_.yaw[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialBaseStates_.pitch: " << initialBaseStates_.pitch[0] << std::endl;
    // std::cout << "[PrePlanning::setUpInitialStates] initialBaseStates_.roll: "  << initialBaseStates_.roll[0] << std::endl;
}

void PrePlanning::setUpGoalStates(const GripperBasePosition& targetPosition) {
    goalBaseStates_.x[0]     = initialBaseStates_.x[0]     + targetPosition.base_x;
    goalBaseStates_.y[0]     = initialBaseStates_.y[0]     + targetPosition.base_y;
    goalBaseStates_.z[0]     = initialBaseStates_.z[0]     + targetPosition.base_z;
    goalBaseStates_.yaw[0]   = initialBaseStates_.yaw[0]   + targetPosition.base_yaw;
    goalBaseStates_.pitch[0] = initialBaseStates_.pitch[0] + targetPosition.base_pitch;
    goalBaseStates_.roll[0]  = initialBaseStates_.roll[0]  + targetPosition.base_roll;

    goalGripperStates_.x[0]     = initialGripperStates_.x[0]     + targetPosition.gripper_x;
    goalGripperStates_.y[0]     = initialGripperStates_.y[0]     + targetPosition.gripper_y;
    goalGripperStates_.z[0]     = initialGripperStates_.z[0]     + targetPosition.gripper_z;
    goalGripperStates_.yaw[0]   = initialGripperStates_.yaw[0]   + targetPosition.gripper_yaw;
    goalGripperStates_.pitch[0] = initialGripperStates_.pitch[0] + targetPosition.gripper_pitch;
    goalGripperStates_.roll[0]  = initialGripperStates_.roll[0]  + targetPosition.gripper_roll;

    // std::cout << "[PrePlanning::setUpGoalStates] goalGripperStates_.x: "     << goalGripperStates_.x[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalGripperStates_.y: "     << goalGripperStates_.y[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalGripperStates_.z: "     << goalGripperStates_.z[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalGripperStates_.yaw: "   << goalGripperStates_.yaw[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalGripperStates_.pitch: " << goalGripperStates_.pitch[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalGripperStates_.roll: "  << goalGripperStates_.roll[0] << std::endl;

    // std::cout << "[PrePlanning::setUpGoalStates] goalBaseStates_.x: "     << goalBaseStates_.x[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalBaseStates_.y: "     << goalBaseStates_.y[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalBaseStates_.z: "     << goalBaseStates_.z[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalBaseStates_.yaw: "   << goalBaseStates_.yaw[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalBaseStates_.pitch: " << goalBaseStates_.pitch[0] << std::endl;
    // std::cout << "[PrePlanning::setUpGoalStates] goalBaseStates_.roll: "  << goalBaseStates_.roll[0] << std::endl;
}

void PrePlanning::setUpGoalTime(const scalar_t& targetTime) {
    timeGoal_ = targetTime;
}

 void PrePlanning::setUpPlanTime(const scalar_t& planTime) {
    timePlan_ = planTime;
 }

void PrePlanning::cubicSplinePlan() {

    planedBaseStates_.x     = cubicTrajectory_d(initialBaseStates_.x[0], goalBaseStates_.x[0], timeGoal_, timePlan_);
    planedBaseStates_.y     = cubicTrajectory_d(initialBaseStates_.y[0], goalBaseStates_.y[0], timeGoal_, timePlan_);
    planedBaseStates_.z     = cubicTrajectory_d(initialBaseStates_.z[0], goalBaseStates_.z[0], timeGoal_, timePlan_);
    planedBaseStates_.yaw   = cubicTrajectory_d(initialBaseStates_.yaw[0], goalBaseStates_.yaw[0], timeGoal_, timePlan_);
    planedBaseStates_.pitch = cubicTrajectory_d(initialBaseStates_.pitch[0], goalBaseStates_.pitch[0], timeGoal_, timePlan_);
    planedBaseStates_.roll  = cubicTrajectory_d(initialBaseStates_.roll[0], goalBaseStates_.roll[0], timeGoal_, timePlan_);

    planedGripperStates_.x     = cubicTrajectory_d(initialGripperStates_.x[0], goalGripperStates_.x[0], timeGoal_, timePlan_);
    planedGripperStates_.y     = cubicTrajectory_d(initialGripperStates_.y[0], goalGripperStates_.y[0], timeGoal_, timePlan_);
    planedGripperStates_.z     = cubicTrajectory_d(initialGripperStates_.z[0], goalGripperStates_.z[0], timeGoal_, timePlan_);
    planedGripperStates_.yaw   = cubicTrajectory_d(initialGripperStates_.yaw[0], goalGripperStates_.yaw[0], timeGoal_, timePlan_);
    planedGripperStates_.pitch = cubicTrajectory_d(initialGripperStates_.pitch[0], goalGripperStates_.pitch[0], timeGoal_, timePlan_);
    planedGripperStates_.roll  = cubicTrajectory_d(initialGripperStates_.roll[0], goalGripperStates_.roll[0], timeGoal_, timePlan_);

    // std::cout << "[PrePlanning::cubicSplinePlan] timeGoal_ " << timeGoal_ << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] timePlan_ " << timePlan_ << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] initialBaseStates_.yaw[0] " << initialBaseStates_.yaw[0] << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] goalBaseStates_.yaw[0] " << goalBaseStates_.yaw[0] << std::endl;


    endEffectorVelocity_.base_x     = planedBaseStates_.x[1];
    endEffectorVelocity_.base_y     = planedBaseStates_.y[1];
    endEffectorVelocity_.base_z     = planedBaseStates_.z[1];
    endEffectorVelocity_.base_yaw   = planedBaseStates_.yaw[1];
    endEffectorVelocity_.base_pitch = planedBaseStates_.pitch[1];
    endEffectorVelocity_.base_roll  = planedBaseStates_.roll[1];

    endEffectorVelocity_.gripper_x     = planedGripperStates_.x[1];
    endEffectorVelocity_.gripper_y     = planedGripperStates_.y[1];
    endEffectorVelocity_.gripper_z     = planedGripperStates_.z[1];
    endEffectorVelocity_.gripper_yaw   = planedGripperStates_.yaw[1];
    endEffectorVelocity_.gripper_pitch = planedGripperStates_.pitch[1];
    endEffectorVelocity_.gripper_roll  = planedGripperStates_.roll[1];

    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.base_x "     << endEffectorVelocity_.base_x     << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.base_y "     << endEffectorVelocity_.base_y     << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.base_z "     << endEffectorVelocity_.base_z     << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.base_yaw "   << endEffectorVelocity_.base_yaw   << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.base_pitch " << endEffectorVelocity_.base_pitch << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.base_yaw "   << endEffectorVelocity_.base_roll  << std::endl;

    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.gripper_x "     << endEffectorVelocity_.gripper_x     << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.gripper_y "     << endEffectorVelocity_.gripper_y     << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.gripper_z "     << endEffectorVelocity_.gripper_z     << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.gripper_yaw "   << endEffectorVelocity_.gripper_yaw   << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.gripper_pitch " << endEffectorVelocity_.gripper_pitch << std::endl;
    // std::cout << "[PrePlanning::cubicSplinePlan] endEffectorVelocity_.gripper_yaw "   << endEffectorVelocity_.gripper_roll  << std::endl;
}

bool PrePlanning::isMotionFinished() {
    if (timeNow_ <= timeGoal_) {
        return false;
    }
    else {
        return true;
    }
}

void PrePlanning::keepMotion(const SystemObservation& observation, const vector_t& defaultLegJointState, TargetTrajectories& trajectory) {

    trajectory.timeTrajectory.resize(2);
    trajectory.stateTrajectory.resize(2);
    trajectory.inputTrajectory.resize(2);

    trajectory.timeTrajectory[0] = observation.time;
    trajectory.stateTrajectory[0] = stateTrajectoryLast_;
    trajectory.inputTrajectory[0] = inputTrajectoryLast_;

    trajectory.timeTrajectory[1] = observation.time + planHorizon_;
    trajectory.stateTrajectory[1] = stateTrajectoryLast_;
    trajectory.inputTrajectory[1] = inputTrajectoryLast_;
    
}

vector3_t PrePlanning::cubicTrajectory_d(float startPoint, float finalPoint, float finalTime, float time_traj) {
    scalar_t a_0, a_1, a_2, a_3;
    vector3_t point_inter;
    a_0 = startPoint;
	a_1 = 0;
	a_2 = 3* (finalPoint - startPoint)/(pow(finalTime,2));
	a_3 = -2* (finalPoint - startPoint)/(powf(finalTime,3));

	point_inter[0] = a_0 + a_1 * time_traj + a_2 *pow(time_traj,2) + a_3 * pow(time_traj,3);//p
	point_inter[1] = a_1 + a_2 * 2 * time_traj + a_3 * 3 * pow(time_traj,2);//v
	point_inter[2] = a_2 * 2 + a_3 * 3 * 2 * time_traj;//a

    return point_inter; 
}

} // namespace arm
} // namespace legged_robot
} // namespace ocs2

