#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

#include "Types.h"

namespace ocs2 {
namespace legged_robot {

class LeggedRobotWithArmVisualizer : public DummyObserver {
    public:
        /** Visualization settings (publicly available) */
        std::string frameId_ = "odom";             // Frame name all messages are published in
        scalar_t limbMarkerDiameter_ = 0.03;       // Size of the spheres at the limb
        scalar_t limbAlphaWhenLifted_ = 0.3;       // Alpha value when a limb is lifted
        scalar_t forceScale_ = 1000.0;             // Vector scale in N/m
        scalar_t velScale = 5.0;                   // Vector scale in m/s
        scalar_t copMarkerDiameter_ = 0.03;        // Size of the sphere at the center of pressure
        scalar_t supportPolygonLineWidth_ = 0.005; // LineThickness for support polygon
        scalar_t trajectoryLineWidth_ = 0.01;      // LineThickness for trajectories
        std::vector<Color> limbsColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple, Color::red}; // Colors for markers per limbs

        /**
         *
         * @param pinocchioInterface
         * @param n
         * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
         */
        LeggedRobotWithArmVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo, 
                                     const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                                     scalar_t maxUpdateFrequency = 100.0);
        
        ~LeggedRobotWithArmVisualizer() override = default;

        void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) override;

        void launchVisualizerNode(ros::NodeHandle& nodeHandle);

        void publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed = 1.0);

        void publishObservation(ros::Time timeStamp, const SystemObservation& observation);

        void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories);

        void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                             const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);
    
    private:
        LeggedRobotWithArmVisualizer(const LeggedRobotWithArmVisualizer&) = delete;
        void publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const;
        void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose);
        void publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags, const std::vector<vector3_t>& limbsPositions,
                                     const std::vector<vector3_t>& limbsForces) const;

        PinocchioInterface pinocchioInterface_;
        const CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

        tf::TransformBroadcaster tfBroadcaster_;
        std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

        ros::Publisher costDesiredBasePositionPublisher_;
        std::vector<ros::Publisher> costDesiredLimbsPositionPublishers_;

        ros::Publisher stateOptimizedPublisher_;

        ros::Publisher currentStatePublisher_;

        scalar_t lastTime_;                        
        scalar_t minPublishTimeDifference_;      
};

} // namespace legged_robot
} // namespace ocs2