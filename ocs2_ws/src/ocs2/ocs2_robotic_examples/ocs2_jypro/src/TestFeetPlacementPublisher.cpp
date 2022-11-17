
#include <ros/ros.h>
#include <random>
#include "ocs2_core/reference/TargetFeetPlacement.h"
#include "ocs2_ros_interfaces/visualization/VisualizationHelpers.h"
#include <visualization_msgs/MarkerArray.h>
#include "ocs2_ros_interfaces/common/RosMsgHelpers.h"

#include <ocs2_msgs/FootholdRegionGroup.h>
#include <ocs2_core/Types.h>

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;

// #define __FOOT_X__ 0.3377
// #define __FOOT_Y__ 0.2033
// #define __FOOT_R__ 0.036

int main(int argc, char** argv) {

  ros::init(argc, argv, "ocs2_jypro_test_publisher");
  ros::NodeHandle nodeHandle;
  ros::Rate rate(5);
  ros::Publisher publisher =
       nodeHandle.advertise<ocs2_msgs::FootholdRegionGroup>("/foothold_planner/FootholdRegionGroup", 1);
  ros::Publisher desiredFeetPlacementPoint_ = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetPlacementPoint", 1);
  
  std::default_random_engine e(3);
      visualization_msgs::Marker feetPlacement;


  std::normal_distribution<ocs2::scalar_t> n(0,0.1);

  std::vector<vector3_t> leftPoints;
  std::vector<vector3_t> rightPoints;

    ocs2_msgs::FootholdRegionGroup msgs;

    msgs.header.stamp = ros::Time::now();
    msgs.header.frame_id = "world";
  
  // while(nodeHandle.ok()) {
    msgs.footholdRegion_LF.clear();
    msgs.footholdRegion_LH.clear();
    msgs.footholdRegion_RF.clear();
    msgs.footholdRegion_RH.clear();
    
    ocs2_msgs::FootholdRegion msg;
    msg.gait_cycle_id = 0;
    msg.rectLength = 0.1;
    
    Eigen::Matrix<ocs2::scalar_t, 3, 1> leftpoint =  {0.0, __FOOT_Y__, __FOOT_R__};
    Eigen::Matrix<ocs2::scalar_t, 3, 1> rightpoint = {0.0, -__FOOT_Y__, __FOOT_R__};

    for(size_t i = 0; i < 100; ++i) {
      if(i < 3){
        leftpoint[0] = 0.25*i - 0.338;
        rightpoint[0] = 0.25*i - 0.338;
        double random = n(e);
        leftpoint[1] += random;
        rightpoint[1] += random;
      }
      else{
        leftpoint[0] = 0.25*(i - 3) + 0.338;
        rightpoint[0] = 0.25*(i - 3) + 0.338;
        // leftpoint[2] = 0.03+0.06*(i-3);
        // rightpoint[2] = 0.03+0.06*(i-3);
        double random = n(e);
        leftpoint[1] += random;
        rightpoint[1] += random;
      }

      if (i == 0 || i == 3)
      {
        leftpoint[1]  = __FOOT_Y__;
        rightpoint[1] = -__FOOT_Y__;
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

      feetPlacement.points.emplace_back(ocs2::getPointMsg(leftpoint));
      feetPlacement.points.emplace_back(ocs2::getPointMsg(rightpoint));
      feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::blue));
      feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::red));
    }
    while (nodeHandle.ok())
    {
      publisher.publish(msgs);
      rate.sleep();
      feetPlacement.header.frame_id = "odom";
      feetPlacement.header.stamp = ros::Time::now();
      feetPlacement.type = visualization_msgs::Marker::SPHERE_LIST;
      feetPlacement.scale.x = 0.03;
      feetPlacement.scale.y = 0.03;
      feetPlacement.scale.z = 0.03;
      feetPlacement.ns = "desired feet placement";
      feetPlacement.pose.orientation = ocs2::getOrientationMsg({1., 0., 0., 0.});

      // feetPlacement.points.emplace_back(ocs2::getPointMsg(leftpoint));
      // feetPlacement.points.emplace_back(ocs2::getPointMsg(rightpoint));
      // feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::blue));
      // feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::red));

          // leftPoints.emplace_back(leftpoint);
          // rightPoints.emplace_back(rightpoint);
      desiredFeetPlacementPoint_.publish(feetPlacement);
    }
           
    
    
  // }

  
  return 0;
}
