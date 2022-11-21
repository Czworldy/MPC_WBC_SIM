// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

//OCS2
#include "MotionPhaseDefinition.h"
#include "LeggedRobotWithArmVisualizer.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmVisualizer::LeggedRobotWithArmVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                                           const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                                                           scalar_t maxUpdateFrequency)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          centroidalModelInfo_(std::move(centroidalModelInfo)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          lastTime_(std::numeric_limits<scalar_t>::lowest()),
          minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
    endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
    launchVisualizerNode(nodeHandle);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
    costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot_with_arm/desiredBaseTrajectory", 1);
    costDesiredLimbsPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
    costDesiredLimbsPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot_with_arm/desiredLimbsTrajectory/LF", 1);
    costDesiredLimbsPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot_with_arm/desiredLimbsTrajectory/RF", 1);
    costDesiredLimbsPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot_with_arm/desiredLimbsTrajectory/LH", 1);
    costDesiredLimbsPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot_with_arm/desiredLimbsTrajectory/RH", 1);
    costDesiredLimbsPositionPublishers_[4] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot_with_arm/desiredLimbsTrajectory/ARM", 1);
    stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot_with_arm/optimizedStateTrajectory", 1);
    currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("legged_robot_with_arm/currentState", 1);

    // Load URDF model
    urdf::Model urdfModel;
    if (!urdfModel.initParam("legged_robot_with_arm_description")) {
        std::cerr << "[LeggedRobotWithArmVisualizer] Could not read URDF from: \"legged_robot_with_arm_description\"" << std::endl;
    } else {
        KDL::Tree kdlTree;
        kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

        robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
        robotStatePublisherPtr_->publishFixedTransforms(true);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto timeStamp = ros::Time::now();
        publishObservation(timeStamp, observation);
        publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
        publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_, 
                                        primalSolution.modeSchedule_);
        lastTime_ = observation.time;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishObservation(ros::Time timeStamp, const SystemObservation& observation) {
    // Extract components from state
    const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
    const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

    // Compute cartesian state and inputs
    const auto limbsPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
    std::vector<vector3_t> limbsForces(centroidalModelInfo_.numThreeDofContacts);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        limbsForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
    }

    // Publish
    publishJointTransforms(timeStamp, qJoints);
    publishBaseTransform(timeStamp, basePose);
    publishCartesianMarkers(timeStamp, modeNumber2StanceLimb(observation.mode), limbsPositions, limbsForces);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const {
    if (robotStatePublisherPtr_ != nullptr) {
        std::map<std::string, scalar_t> jointPositions{{"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
                                                       {"LH_HAA", jointAngles[3]}, {"LH_HFE", jointAngles[4]},  {"LH_KFE", jointAngles[5]},
                                                       {"RF_HAA", jointAngles[6]}, {"RF_HFE", jointAngles[7]},  {"RF_KFE", jointAngles[8]},
                                                       {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]},
                                                       {"ARM_J1", jointAngles[12]}, {"ARM_J2", jointAngles[13]}, {"ARM_J3", jointAngles[14]},{"ARM_J4", jointAngles[15]}
                                                       ,{"ARM_J5", jointAngles[16]},{"ARM_J6", jointAngles[17]}};
        robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t& basePose) {
    if (robotStatePublisherPtr_ != nullptr) {
        geometry_msgs::TransformStamped baseToWorldTransform;
        baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
        baseToWorldTransform.child_frame_id = "base";

        const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
        baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
        baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
        tfBroadcaster_.sendTransform(baseToWorldTransform);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed) {
    for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
        scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
        scalar_t publishDuration = timedExecutionInSeconds([&]() { publishObservation(ros::Time::now(), system_observation_array[k]); });
        if (frameDuration > publishDuration) {
          ros::WallDuration(frameDuration - publishDuration).sleep();
        }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags,
                                                           const std::vector<vector3_t>& limbsPositions,
                                                           const std::vector<vector3_t>& limbsForces) const {
    // Reserve message
    const size_t numberOfCartesianMarkers = 12;
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.reserve(numberOfCartesianMarkers);

    // Limbs positions and Forces
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
        markerArray.markers.emplace_back(
                getFootMarker(limbsPositions[i], contactFlags[i], limbsColorMap_[i], limbMarkerDiameter_, limbAlphaWhenLifted_));
        markerArray.markers.emplace_back(getForceMarker(limbsForces[i], limbsPositions[i], contactFlags[i], Color::green, forceScale_));
    }

    // Center of pressure
    markerArray.markers.emplace_back(getCenterOfPressureMarker(limbsForces.begin(), limbsForces.end(), limbsPositions.begin(), 
                                                               contactFlags.begin(), Color::green, copMarkerDiameter_));

    // Support polygon
    markerArray.markers.emplace_back(
            getSupportPolygonMarker(limbsPositions.begin(), limbsPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

    // Give markers an id and a frame
    assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

    // Publish cartesian markers (minus the CoM Pose)
    currentStatePublisher_.publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories) {
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;
    const auto& inputTrajectory = targetTrajectories.inputTrajectory;
    const auto& eePositionTrajectory = targetTrajectories.eePositionTrajectory;

    // Reserve com messages
    std::vector<geometry_msgs::Point> desiredBasePositionMsg;
    desiredBasePositionMsg.reserve(stateTrajectory.size());

    // Reserve limbs messages
    feet_arm_array_t<std::vector<geometry_msgs::Point>> desiredLimbsPositionMsgs;
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        desiredLimbsPositionMsgs[i].reserve(stateTrajectory.size());
    }

    for (size_t j = 0; j < stateTrajectory.size(); j++) {
        const auto state = stateTrajectory.at(j);
        vector_t input(centroidalModelInfo_.inputDim);
        if (j < inputTrajectory.size()) {
            input = inputTrajectory.at(j);
        } else {
            input.setZero();
        }

        // Construct base pose msg
        const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
        geometry_msgs::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());

        // Fill message containers
        desiredBasePositionMsg.push_back(pose.position);

        // Fill limbs msgs
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto limbsPositions = endEffectorKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            geometry_msgs::Pose limbPose;
            limbPose.position = getPointMsg(limbsPositions[i]);
            desiredLimbsPositionMsgs[i].push_back(limbPose.position);
            
        }

    }

    // Headers
    auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
    comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    comLineMsg.id = 0;

    // Publish
    costDesiredBasePositionPublisher_.publish(comLineMsg);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        auto limbLineMsg = getLineMsg(std::move(desiredLimbsPositionMsgs[i]), limbsColorMap_[i], trajectoryLineWidth_);
        limbLineMsg.header = getHeaderMsg(frameId_, timeStamp);
        limbLineMsg.id = 0;
        costDesiredLimbsPositionPublishers_[i].publish(limbLineMsg);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                                   const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule) {
    if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
        return; // Nothing to publish
    }

    // Reserve Limbs Msg
    feet_arm_array_t<std::vector<geometry_msgs::Point>> limbsMsgs;
    std::for_each(limbsMsgs.begin(), limbsMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

    // Reserve Com Msg
    std::vector<geometry_msgs::Point> mpcComPositionMsgs;
    mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

    // Extract Com and Limbs from state
    std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
        const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
         
        // Fill com position and pose msgs
        geometry_msgs::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());
        mpcComPositionMsgs.push_back(pose.position);

        // Fill feet msgs
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto limbsPositions = endEffectorKinematicsPtr_->getPosition(state);
        for(size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            const auto position = getPointMsg(limbsPositions[i]);
            limbsMsgs[i].push_back(position);
        }
    });

    // Convert feet msgs to Array message
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts + 
                                2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        markerArray.markers.emplace_back(getLineMsg(std::move(limbsMsgs[i]), limbsColorMap_[i], trajectoryLineWidth_));
        markerArray.markers.back().ns = "EE Trajectories";
    }
    markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
    markerArray.markers.back().ns = "CoM Trajectory";

    // Future footholds
    visualization_msgs::Marker sphereList;
    sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereList.scale.x = limbMarkerDiameter_;
    sphereList.scale.y = limbMarkerDiameter_;
    sphereList.scale.z = limbMarkerDiameter_;
    sphereList.ns = "Future footholds";
    sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
    const auto& eventTimes = modeSchedule.eventTimes;
    const auto& subsystemSequence = modeSchedule.modeSequence;
    const auto tStart = mpcTimeTrajectory.front();
    const auto tEnd = mpcTimeTrajectory.back();
    for(size_t event = 0; event < eventTimes.size(); ++event) {
        if(tStart < eventTimes[event] && eventTimes[event] < tEnd) { // Only publish future footholds within the optimized horizon
            const auto preEventContactFlags = modeNumber2StanceLimb(subsystemSequence[event]);
            const auto postEventContactFlags = modeNumber2StanceLimb(subsystemSequence[event + 1]);
            const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);
            
            const auto& model = pinocchioInterface_.getModel();
            auto& data = pinocchioInterface_.getData();
            pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
            pinocchio::updateFramePlacements(model, data);

            const auto limbsPositions = endEffectorKinematicsPtr_->getPosition(postEventState);
            for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts - 1; i++) {
                if (!preEventContactFlags[i] && postEventContactFlags[i]) { // If a foot lands, a marker is added at that location.
                    sphereList.points.emplace_back(getPointMsg(limbsPositions[i]));
                    sphereList.colors.push_back(getColor(limbsColorMap_[i]));
                }
            }
        }
    }
    markerArray.markers.push_back(std::move(sphereList));

    // Add headers and Id
    assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

    stateOptimizedPublisher_.publish(markerArray);
}

} // namespace legged_robot
} // namespace ocs2