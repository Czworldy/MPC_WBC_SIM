#include "ocs2_jypro/visualization/FootPlacementVisualizer.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>


namespace ocs2 {
namespace legged_robot {

FootPlacementVisualizer::FootPlacementVisualizer(ros::NodeHandle& nodeHandle, const SwingTrajectoryPlanner& SwingTrajectoryPlanner) :
  swingTrajectoryPlannerPtr_(&SwingTrajectoryPlanner)
  {
    footPlacementPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/Footplacement", 1);
}

FootPlacementVisualizer::~FootPlacementVisualizer(){}

void FootPlacementVisualizer::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                            const ReferenceManagerInterface& referenceManager){
  visualization_msgs::MarkerArray footPlacementMarkerArray;
  footPlacementMarkerArray.markers.reserve(4);
  feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
  // std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });
  const vector_t currentPose = initState.segment<6>(6);
  const scalar_t currentYaw = currentPose(3);
  matrix_t tfMatrix = matrix_t::Identity(4, 4);
  const legged_robot::vector3_t ZyxEulerAngles = currentPose.tail(3);
  std::cout << "ZyxEulerAngles: " << ZyxEulerAngles.transpose() << "\n";
  legged_robot::matrix3_t rotationMatrix = ocs2::getRotationMatrixFromZyxEulerAngles(ZyxEulerAngles);

  tfMatrix.topLeftCorner(3, 3) = rotationMatrix;
  tfMatrix.topRightCorner(3, 1) = currentPose.head(3);

  for (int i = 0; i < 4; i++){
    for(scalar_t time = initTime; time < finalTime; time += 0.01){
      vector3_t position;
      position << swingTrajectoryPlannerPtr_->getXpositionConstraint(i, time),
                  swingTrajectoryPlannerPtr_->getYpositionConstraint(i, time),
                  swingTrajectoryPlannerPtr_->getZpositionConstraint(i, time);
      const vector3_t tfedPoint = (tfMatrix.inverse()*position.homogeneous()).head(3);
      const auto point = getPointMsg(tfedPoint);
      feetMsgs[i].push_back(point);
    }
    auto line = getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    line.header.frame_id = "base_footprint";

    footPlacementMarkerArray.markers.emplace_back(line);
    footPlacementMarkerArray.markers.back().ns = "EE design Trajectories";
  }

  assignHeader(footPlacementMarkerArray.markers.begin(), footPlacementMarkerArray.markers.end(), getHeaderMsg(frameId_, ros::Time::now()));
  assignIncreasingId(footPlacementMarkerArray.markers.begin(), footPlacementMarkerArray.markers.end());

  footPlacementPublisher_.publish(footPlacementMarkerArray);
}


}
}