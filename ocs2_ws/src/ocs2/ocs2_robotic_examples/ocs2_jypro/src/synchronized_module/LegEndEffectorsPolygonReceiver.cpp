#include "ocs2_jypro/synchronized_module/LegEndEffectorsPolygonReceiver.h"
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>


namespace ocs2 {
namespace legged_robot {

LegEndEffectorsPolygonReceiver::LegEndEffectorsPolygonReceiver(ros::NodeHandle nodeHandle,
                                    std::shared_ptr<feet_polygon_array_t> mpcPolygonsPtr,
                                    std::shared_ptr<feet_array_t<std::vector<vector3_t>>> mpcNominalFeetholdsPtr,
                                    const std::string& robotName) :
  mpcTransformedPolygonsPtr_(std::move(mpcPolygonsPtr)),
  mpcTransformedNominalFeetholdsPtr_(std::move(mpcNominalFeetholdsPtr)) {

  receivedFeetPoints_ = *mpcTransformedPolygonsPtr_;
  receivedNominalFeethold_ = *mpcTransformedNominalFeetholdsPtr_;
  mpcPolygonMsgSubscriber_[0] = nodeHandle.subscribe("foothold_planner/RegionForFoot_LF", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  mpcPolygonMsgSubscriber_[1] = nodeHandle.subscribe("foothold_planner/RegionForFoot_RF", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  mpcPolygonMsgSubscriber_[2] = nodeHandle.subscribe("foothold_planner/RegionForFoot_LH", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  mpcPolygonMsgSubscriber_[3] = nodeHandle.subscribe("foothold_planner/RegionForFoot_RH", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>("legged_robot_mpc_observation", 1, observationCallback);

}

inline matrix3_t rpyTORotateMat(double roll, double pitch, double yaw){
    matrix3_t RotateMatrix, R_roll, R_pitch, R_yaw;
    R_roll << 1., 0., 0.,
        0., cos(roll), -sin(roll),
        0., sin(roll), cos(roll);
    R_pitch << cos(pitch), 0, sin(pitch),
        0., 1., 0.,
        -sin(pitch), 0., cos(pitch);
    R_yaw << cos(yaw), -sin(yaw), 0.,
        sin(yaw), cos(yaw), 0.,
        0., 0., 1.;
    RotateMatrix = R_yaw * R_pitch * R_roll;
    return RotateMatrix;
}

void LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback(const ocs2_msgs::RegionForFoot::ConstPtr& msg){
  std::lock_guard<std::mutex> lock(receivedPolygonMsgMutex_);
  // legEndeffectorPolygonReceived_->clear();
  // for (int i = 0; i < msg->polygons.size(); i++){
  //   legEndeffectorPolygonReceived_->push_back(ocs2::Polygon(msg->polygons[i]));
  // }
  const int foot_id = msg->foot_id;
  // std::cout << "foot_id Constraint Callback: " << foot_id << std::endl;
  receivedFeetPoints_[foot_id].clear();
  receivedFeetPoints_[foot_id].reserve(msg->region.size());
  receivedNominalFeethold_[foot_id].clear();
  receivedNominalFeethold_[foot_id].reserve(msg->region.size());

  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  const vector_t currentPose = observation.state.segment<6>(6);
  matrix_t tfMatrix = matrix_t::Identity(4,4);
  const vector3_t ZyxEulerAngles = currentPose.tail(3);
  const matrix3_t rotationMatrix = rpyTORotateMat(currentPose(5), currentPose(4), currentPose(3));
  tfMatrix.topLeftCorner(3, 3) = rotationMatrix;
  tfMatrix.topRightCorner(3,1) = currentPose.head(3);

  // std::cout << "msg size:" << msg->region.size() << " " << msg->region[0].boundaryPoint.size() << "\n";
  for (int i = 0; i < msg->region.size(); i++) {
      std::vector<Eigen::Vector3d> polygon;
      polygon.reserve(msg->region[i].boundaryPoint.size());
      for (int j = 0; j < msg->region[i].boundaryPoint.size(); j++) {
          const vector3_t point = {msg->region[i].boundaryPoint[j].x, msg->region[i].boundaryPoint[j].y, msg->region[i].boundaryPoint[j].z};
          const auto& transformedPoint = tfMatrix * point.homogeneous();
          polygon.push_back(transformedPoint.head(3));
      }
      receivedFeetPoints_[foot_id].push_back(polygon);
      const vector3_t nominalPoint = {msg->region[i].nominalFoothold.x, msg->region[i].nominalFoothold.y, msg->region[i].nominalFoothold.z};
      const auto& transformedNominalPoint = tfMatrix * nominalPoint.homogeneous();
      receivedNominalFeethold_[foot_id].push_back(transformedNominalPoint.head(3));
  }

    polygonsUpdated_ = true;
}

void LegEndEffectorsPolygonReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                            const ReferenceManagerInterface& referenceManager) {
  if(polygonsUpdated_){
    std::lock_guard<std::mutex> lock(receivedPolygonMsgMutex_);
    // std::cout << "polygonsUpdated_\n";
    // point transformed to the world frame. 4x4 tf matrix.
    // const auto& currentPose = initState.segment<6>(6);
    // matrix_t tfMatrix = matrix_t::Identity(4,4);
    // const vector3_t ZyxEulerAngles = currentPose.tail(3);
    // const matrix3_t rotationMatrix = rpyTORotateMat(currentPose(5), currentPose(4), currentPose(3));
    // // tfMatrix.topLeftCorner(3, 3) = rotationMatrix;
    // // tfMatrix.topRightCorner(3,1) = currentPose.head(3);

    //     // std::cout << "TF: " << tfMatrix << "\n";

    //     transformedFeetPoints_ = receivedFeetPoints_; // this is deep copy.

    //     // transformedPolygons_

    // size_t leg = 0, polygonIndex = 0, pointIndex = 0;
    // for(const auto& polygons:receivedFeetPoints_){
    //   // transformedPolygons_[leg].clear();
    //   // transformedPolygons_[leg].reserve(polygons.size());
    //   polygonIndex = 0;
    //   for(const auto& polygon:polygons){
    //     // transformedPolygons_[leg][polygonIndex].reserve(polygon.size());
    //     std::vector<ocs2::Polygon::Position> xyPoints;
    //     xyPoints.reserve(polygon.size());
    //     pointIndex = 0;
    //     for(const auto& point:polygon){
    //       const auto& transformedPoint = tfMatrix * point.homogeneous();
    //       transformedFeetPoints_[leg][polygonIndex][pointIndex] = transformedPoint.matrix().topRows(3);
    //       xyPoints.push_back(transformedPoint.matrix().topRows(2));
    //       ++pointIndex;
    //     }
    //     // transformedPolygons_[leg].emplace_back(xyPoints);
    //     ++polygonIndex;
    //   }
    //   ++leg;
    // }

    // transformedNominalFeethold_ = receivedNominalFeethold_;
    // leg = 0;
    // for(const auto& footholds:receivedNominalFeethold_){
    //   polygonIndex = 0;
    //   for(const auto& point:footholds){
    //     const auto& transformedPoint = tfMatrix * point.homogeneous();
    //     transformedNominalFeethold_[leg][polygonIndex] = transformedPoint.matrix().topRows(3);
    //     ++polygonIndex;
    //   }
    //   ++leg;
    // }

    *mpcTransformedPolygonsPtr_ = receivedFeetPoints_;
    *mpcTransformedNominalFeetholdsPtr_ = receivedNominalFeethold_;

        // save the transformed points to let footplacementplanner chose.
        // create polygon using transformed points.
        // form the constraints. (cteate constraints struct)

        // legEndeffectorPolygonReceived_->copy(terrainEstDataTemp_);
        polygonsUpdated_ = false;
    }
}

LegEndEffectorsPolygonReceiver::~LegEndEffectorsPolygonReceiver() {}

} // namespace legged_robot
} // namespace ocs2
