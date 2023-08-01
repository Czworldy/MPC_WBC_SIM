#include <ros/ros.h>
#include <random>

#include "ocs2_ros_interfaces/visualization/VisualizationHelpers.h"
#include <visualization_msgs/MarkerArray.h>
#include "ocs2_ros_interfaces/common/RosMsgHelpers.h"
// #include "ocs2_jypro/synchronized_module/TerrainReceiver.h"

#include <ocs2_msgs/FootholdRegionGroup.h>
#include <ocs2_core/Types.h>
#include <nav_msgs/Odometry.h>

// #include <stdio.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <netinet/in.h>

// #define SERVER_PORT 2222
// #define SERVER_IP "127.0.0.1"

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;

// Struct
// struct LimbsContacts {
// public: 
//     float lf;
//     float rf;
//     float lh;
//     float rh;
// };
// typedef struct
// {
// 	double value[3];
// } OneLimbData;

// typedef struct
// {
// 	OneLimbData lf_pos;
// 	OneLimbData rf_pos;
// 	OneLimbData lh_pos;
// 	OneLimbData rh_pos;
// 	OneLimbData lf_vel;
// 	OneLimbData rf_vel;
// 	OneLimbData lh_vel;
// 	OneLimbData rh_vel;
// }	LimbsPosVel;

// struct EstimatorOutput {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// public:
//     float time_stamp;
//     Eigen::Matrix<double, 3, 1> base_pos_world;
//     Eigen::Matrix<double, 3, 1> base_linear_vel_world;
//     Eigen::Matrix<double, 3, 1> base_linear_vel_body;
//     Eigen::Quaterniond base_orientation_world;
//     Eigen::Matrix<double, 3, 1> base_angular_vel_world;
//     Eigen::Matrix<double, 3, 1> base_angular_vel_body;
//     LimbsContacts contact;
//     LimbsPosVel jointStates;

//     Eigen::Matrix<double, 3, 1> frame_c_rpy_in_world;
//     Eigen::Quaterniond frame_c_quat_in_world;
//     Eigen::Matrix<double, 3, 1> frame_c_xyz_in_world;

//     ocs2::legged_robot::TerrainEstData terrainEstData;
// #ifdef USE_TERRAIN    
//     std::vector<Eigen::Vector3d> foot_position;
//     Eigen::Quaterniond terrain_orientation;
//     Eigen::Vector3d terrain_params;
// #endif
// };

// #define __FOOT_X__ 0.3377
// #define __FOOT_Y__ 0.2033
// #define __FOOT_R__ 0.036
Eigen::Matrix4d transformMat, transformMat_odom2map;
void liderMsgCallback(const nav_msgs::Odometry::ConstPtr& msg){
  vector3_t pose;
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
  pose[2] = msg->pose.pose.position.z;

  Eigen::Quaterniond quat;
  quat.x() = msg->pose.pose.orientation.x;
  quat.y() = msg->pose.pose.orientation.y;
  quat.z() = msg->pose.pose.orientation.z;
  quat.w() = msg->pose.pose.orientation.w;

  Eigen::Matrix3d R = quat.toRotationMatrix();

  transformMat.setZero();
  transformMat.topLeftCorner(3, 3) = R;
  transformMat.col(3) << pose, 1;
  std::cout << "TF:-----------\n" << transformMat << "-----------\n";

}

void map2OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  vector3_t pose;
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
  pose[2] = msg->pose.pose.position.z;

  Eigen::Quaterniond quat;
  quat.x() = msg->pose.pose.orientation.x;
  quat.y() = msg->pose.pose.orientation.y;
  quat.z() = msg->pose.pose.orientation.z;
  quat.w() = msg->pose.pose.orientation.w;

  Eigen::Matrix3d R = quat.toRotationMatrix();

  transformMat_odom2map.setZero();
  transformMat_odom2map.topLeftCorner(3, 3) = R;
  transformMat_odom2map.col(3) << pose, 1;
  std::cout << "TFodom2map:-----------\n" << transformMat_odom2map << "-----------\n";
}



int main(int argc, char** argv) {

    //FOR UDP CLIENT
    // int rec_fd;
    // struct sockaddr_in rec_aadr;
    // rec_fd = socket(AF_INET, SOCK_DGRAM, 0);
    // if(rec_fd < 0){
    //     printf("create socket fail!\n");
    //     return -1;
    // }
    // memset(&rec_aadr, 0 , sizeof(rec_aadr));
    // rec_aadr.sin_family = AF_INET;
    // rec_aadr.sin_addr.s_addr = htonl(INADDR_ANY);
    // rec_aadr.sin_port = htons(SERVER_PORT);
    // EstimatorOutput buf;
    // memset(&buf, 0, sizeof(EstimatorOutput));
    // socklen_t len;
    // size_t buf_len;
    // len = sizeof(rec_aadr);
    // buf_len = sizeof(buf);
    // std::cout << buf_len << "\n";
    // if (bind(rec_fd, (sockaddr *)&rec_aadr, sizeof(rec_aadr)) == -1) {
    //     std::cerr << ">>>>>>>>>>>>>>>>>>UDPInit:time bind failed:\n";
    //     return 3;
    // }
    // std::cout << "UDP init done!\n";

  ros::init(argc, argv, "ocs2_jypro_test_publisher");
  ros::NodeHandle nodeHandle;
  ros::Rate rate(5);
  ros::Publisher publisher =
       nodeHandle.advertise<ocs2_msgs::FootholdRegionGroup>("/foothold_planner/FootholdRegionGroup", 1);
  // ros::Publisher desiredFeetPlacementPoint_ = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetPlacementPoint", 1);
  ros::Subscriber sub_tf = nodeHandle.subscribe<nav_msgs::Odometry>("/base2odometry", 1, &liderMsgCallback);
  ros::Subscriber sub_odometry2map = nodeHandle.subscribe<nav_msgs::Odometry>("/odometry2map", 1, &map2OdomCallback);
  
  std::default_random_engine e(3);
      visualization_msgs::Marker feetPlacement;


  std::normal_distribution<ocs2::scalar_t> n(0,0.1);

  std::vector<vector3_t> leftPoints;
  std::vector<vector3_t> rightPoints;

  Eigen::Matrix4d transformFix_odom2s ,transformFix_Lidar2base;
  transformFix_Lidar2base << -1, 0, 0, -0,
                              0,-1, 0,  0,
                              0, 0, 1, -0,
                              0, 0, 0,  1;
  transformFix_odom2s     << -1, 0, 0, 0,
                              0,-1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1;
  
  std::cout << "transformFix_odom2s:--------\n " << transformFix_odom2s << "--------\n";
  std::cout << "transformFix_Lidar2base:--------\n " << transformFix_Lidar2base << "--------\n";
  transformMat.setIdentity();
  transformMat_odom2map.setIdentity();

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
    
    vector3_t leftpoint =  {0.0, __FOOT_Y__, __FOOT_R__};
    vector3_t rightpoint = {0.0, -__FOOT_Y__, __FOOT_R__};

    for(size_t i = 0; i < 23; ++i) {
      if(i < 2){
        leftpoint[0] = 0.3*i - __FOOT_X__;
        rightpoint[0] = 0.3*i - __FOOT_X__;
        double random = n(e);
        leftpoint[1] += random;
        rightpoint[1] += random;
      }
      else{
        leftpoint[0] = 0.3*(i - 2) + __FOOT_X__;
        rightpoint[0] = 0.3*(i - 2) + __FOOT_X__;
        // leftpoint[2] = 0.03+0.06*(i-3);
        // rightpoint[2] = 0.03+0.06*(i-3);
        // double random = n(e);
        // leftpoint[1] += random;
        // rightpoint[1] += random;
      }
      // if(i > 5 && i < 10){
      //   leftpoint[0] = 1 + 0.23*(i -5) - 0.05;
      //   rightpoint[0] = 1 + 0.23*(i -5) - 0.05;
      //   leftpoint[1] = __FOOT_Y__;
      //   rightpoint[1] = -__FOOT_Y__;
      //   leftpoint[2] = 0.08 * (i-5);
      //   rightpoint[2] = 0.08 * (i-5);

      // }

      // if(i >= 10){
      //   leftpoint[0] = 1.87 + 0.25*(i - 9);
      //   rightpoint[0] = 1.87 + 0.25*(i - 9);

      //   leftpoint[1] = __FOOT_Y__;
      //   rightpoint[1] = -__FOOT_Y__;

      //   leftpoint[2] = 0.08 * 4;
      //   rightpoint[2] = 0.08 * 4;
      // }

      if (i == 0 || i == 2)
      {
        leftpoint[1]  = __FOOT_Y__;
        rightpoint[1] = -__FOOT_Y__;
      }
      leftPoints.push_back(leftpoint);
      rightPoints.push_back(rightpoint);
      auto draw_leftpoint = leftpoint;
      auto draw_rightpoint = rightpoint;
      std::cout << "L_point Y:\t" << draw_leftpoint[1]+0.18-0.09 << "\n";
      std::cout << "R_point Y:\t" << draw_rightpoint[1]+0.18-0.09 << "\n";

      msg.rectCenter_Position.x = leftpoint[0];
      msg.rectCenter_Position.y = leftpoint[1];
      msg.rectCenter_Position.z = leftpoint[2];
      // msgs.footholdRegion_LF.emplace_back(msg);
      // msgs.footholdRegion_LH.emplace_back(msg);

      msg.rectCenter_Position.x = rightpoint[0];
      msg.rectCenter_Position.y = rightpoint[1];
      msg.rectCenter_Position.z = rightpoint[2];

      // msgs.footholdRegion_RF.emplace_back(msg);
      // msgs.footholdRegion_RH.emplace_back(msg);

      feetPlacement.points.emplace_back(ocs2::getPointMsg(leftpoint));
      feetPlacement.points.emplace_back(ocs2::getPointMsg(rightpoint));
      feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::blue));
      feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::red));
    }
    while (nodeHandle.ok())
    {
      // recvfrom(rec_fd, &buf, buf_len, 0, (struct sockaddr*)&rec_aadr, &len);
      // vector3_t ekf_point = buf.base_pos_world;
      // Eigen::Quaterniond ekf_quat = buf.base_orientation_world;
      ros::spinOnce();
      msgs.footholdRegion_LF.clear();
      msgs.footholdRegion_LH.clear();
      msgs.footholdRegion_RF.clear();
      msgs.footholdRegion_RH.clear();
      for(int i = 0; i < leftPoints.size(); i++){
        Eigen::Vector4d temp, res;
        temp << leftPoints[i], 1;
        std::cout << "Ltemp: " << temp.transpose() << "\n";
        res = transformFix_Lidar2base * transformMat.inverse() * transformMat_odom2map * transformFix_odom2s * temp;
        std::cout << "Lres: " << res.transpose() << "\n";
        msg.rectCenter_Position.x = res[0];
        msg.rectCenter_Position.y = res[1];
        msg.rectCenter_Position.z = res[2];
        msgs.footholdRegion_LF.emplace_back(msg);
        msgs.footholdRegion_LH.emplace_back(msg);
        
        temp << rightPoints[i], 1;
        std::cout << "Rtemp: " << temp.transpose() << "\n";
        res = transformFix_Lidar2base * transformMat.inverse() * transformMat_odom2map * transformFix_odom2s * temp;
        std::cout << "Rres: " << res.transpose() << "\n";

        msg.rectCenter_Position.x = res[0];
        msg.rectCenter_Position.y = res[1];
        msg.rectCenter_Position.z = res[2];
        msgs.footholdRegion_RF.emplace_back(msg);
        msgs.footholdRegion_RH.emplace_back(msg);
      }

      publisher.publish(msgs);
      rate.sleep(); 
      // feetPlacement.header.frame_id = "odom";
      // feetPlacement.header.stamp = ros::Time::now();
      // feetPlacement.type = visualization_msgs::Marker::SPHERE_LIST;
      // feetPlacement.scale.x = 0.03;
      // feetPlacement.scale.y = 0.03;
      // feetPlacement.scale.z = 0.03;
      // feetPlacement.ns = "desired feet placement";
      // feetPlacement.pose.orientation = ocs2::getOrientationMsg({1., 0., 0., 0.});

      // feetPlacement.points.emplace_back(ocs2::getPointMsg(leftpoint));
      // feetPlacement.points.emplace_back(ocs2::getPointMsg(rightpoint));
      // feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::blue));
      // feetPlacement.colors.push_back(ocs2::getColor(ocs2::Color::red));

          // leftPoints.emplace_back(leftpoint);
          // rightPoints.emplace_back(rightpoint);
      // desiredFeetPlacementPoint_.publish(feetPlacement);
    }
           
    
    
  // }

  
  return 0;
}