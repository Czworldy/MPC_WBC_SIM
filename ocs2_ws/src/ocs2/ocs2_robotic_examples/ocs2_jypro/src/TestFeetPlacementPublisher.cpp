
#include <ros/ros.h>
#include <random>
#include "ocs2_core/reference/TargetFeetPlacement.h"
#include <ocs2_msgs/FootholdRegionGroup.h>
#include <ocs2_core/Types.h>

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;

int main(int argc, char** argv) {

  ros::init(argc, argv, "ocs2_jypro_test_publisher");
  ros::NodeHandle nodeHandle;
  ros::Rate rate(5);
  ros::Publisher publisher =
       nodeHandle.advertise<ocs2_msgs::FootholdRegionGroup>("legged_robot_target_feet_placement", 1);
  std::default_random_engine e(2);

  std::normal_distribution<ocs2::scalar_t> n(0,0.01);

  std::vector<vector3_t> leftPoints;
  std::vector<vector3_t> rightPoints;

    ocs2_msgs::FootholdRegionGroup msgs;

    msgs.header.stamp = ros::Time::now();
    msgs.header.frame_id = "world";
  
  while(nodeHandle.ok()) {
    msgs.footholdRegion_LF.clear();
    msgs.footholdRegion_LH.clear();
    msgs.footholdRegion_RF.clear();
    msgs.footholdRegion_RH.clear();
    
    ocs2_msgs::FootholdRegion msg;
    msg.gait_cycle_id = 0;
    msg.rectLength = 0.1;
    

    for(size_t i = 0; i < 10; ++i) {
      Eigen::Matrix<ocs2::scalar_t, 3, 1> leftpoint = {-0.177, 0.0, 0.03};
      Eigen::Matrix<ocs2::scalar_t, 3, 1> rightpoint = {0.177, 0.0, 0.03};
      if(i < 3){
        leftpoint[1] = 0.25*i - 0.338;
        rightpoint[1] = 0.25*i - 0.338;
        leftpoint[0] += n(e);
        rightpoint[0] += n(e);
      }
      else{
        leftpoint[1] = 0.25*(i - 3) + 0.338;
        rightpoint[1] = 0.25*(i - 3) + 0.338;
        // leftpoint[2] = 0.03+0.06*(i-3);
        // rightpoint[2] = 0.03+0.06*(i-3);
        leftpoint[0] += n(e);
        rightpoint[0] += n(e);
      }

      if (i == 0 || i == 3)
      {
        leftpoint[0] = -0.177;
        rightpoint[0] = 0.177;
      }
      msg.rectCenter_Position.x = leftpoint[0];
      msg.rectCenter_Position.y = leftpoint[1];
      msg.rectCenter_Position.z = leftpoint[2];
      msgs.footholdRegion_LF.emplace_back(msg);
      msgs.footholdRegion_LH.emplace_back(msg);

      msg.rectCenter_Position.x = rightpoint[0];
      msg.rectCenter_Position.y = rightpoint[1];
      msg.rectCenter_Position.z = rightpoint[2];

      msgs.footholdRegion_RF.emplace_back(msg);
      msgs.footholdRegion_RH.emplace_back(msg);
    }
    publisher.publish(msgs);
    rate.sleep();
  }

  
  return 0;
}
