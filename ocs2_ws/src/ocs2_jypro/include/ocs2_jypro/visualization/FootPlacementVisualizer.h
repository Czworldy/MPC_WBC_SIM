#include "ros/ros.h"
#include "ocs2_jypro/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_ros_interfaces/visualization/VisualizationHelpers.h"
#include <visualization_msgs/MarkerArray.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>




namespace ocs2 {
namespace legged_robot {

class FootPlacementVisualizer : public SolverSynchronizedModule
{
private:
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  ros::Publisher footPlacementPublisher_;
public:
  FootPlacementVisualizer(ros::NodeHandle& nodeHandle, const SwingTrajectoryPlanner& SwingTrajectoryPlanner);
  ~FootPlacementVisualizer();

  std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};  // Colors for markers per feet
  scalar_t trajectoryLineWidth_ = 0.03; 
  std::string frameId_ = "odom";              // Frame name all messages are published in


  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                            const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{}; 

};

}
}