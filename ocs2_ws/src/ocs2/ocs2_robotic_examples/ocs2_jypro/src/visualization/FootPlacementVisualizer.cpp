#include "ocs2_jypro/visualization/FootPlacementVisualizer.h"

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

  for (int i = 0; i < 4; i++){
    for(scalar_t time = initTime; time < finalTime; time += 0.01){
      vector3_t position;
      position << swingTrajectoryPlannerPtr_->getXpositionConstraint(i, time),
                  swingTrajectoryPlannerPtr_->getYpositionConstraint(i, time),
                  swingTrajectoryPlannerPtr_->getZpositionConstraint(i, time);
      const auto point = getPointMsg(position);
      feetMsgs[i].push_back(point);
    }
<<<<<<< HEAD
    auto line = getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    line.header.frame_id = "BASE";
=======
    const auto& line = getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    line.header.frame_id = "BASE";

>>>>>>> b9fcf3f96f398d9ece284f0e58b957af48416962
    footPlacementMarkerArray.markers.emplace_back(line);
    footPlacementMarkerArray.markers.back().ns = "EE design Trajectories";
  }

  assignHeader(footPlacementMarkerArray.markers.begin(), footPlacementMarkerArray.markers.end(), getHeaderMsg(frameId_, ros::Time::now()));
  assignIncreasingId(footPlacementMarkerArray.markers.begin(), footPlacementMarkerArray.markers.end());

  footPlacementPublisher_.publish(footPlacementMarkerArray);
}


}
}