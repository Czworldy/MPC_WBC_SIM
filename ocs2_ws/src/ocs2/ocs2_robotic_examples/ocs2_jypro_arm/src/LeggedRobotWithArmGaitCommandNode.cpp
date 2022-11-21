#include <LeggedRobotWithArmModeSequenceKeyboard.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  ros::init(argc, argv, robotName + "_mpc_mode_schedule");
  ros::NodeHandle nodeHandle;

  std::string gaitFile;
  nodeHandle.getParam("/gaitCommandFile", gaitFile);

  std::cerr << "Loading gait file: " << gaitFile << std::endl;



  LeggedRobotWithArmModeSequenceKeyboard modeSequenceCommand(nodeHandle, gaitFile, robotName, true);

  while (ros::ok() && ros::master::check()) {
    modeSequenceCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}