#include "ocs2_jypro/synchronized_module/LegEndEffectorsPolygonReceiver.h"

namespace ocs2 {
namespace legged_robot {

LegEndEffectorsPolygonReceiver::LegEndEffectorsPolygonReceiver(ros::NodeHandle nodeHandle,
                                                               std::shared_ptr<feet_polygon_array_t> mpcPolygonsPtr,
                                                               const std::string& robotName) :
  mpcTransformedPolygonsPtr_(std::move(mpcPolygonsPtr))  {
  mpcPolygonMsgSubscriber_[0] = nodeHandle.subscribe("foothold_planner/RegionForFoot_LF", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  mpcPolygonMsgSubscriber_[1] = nodeHandle.subscribe("foothold_planner/RegionForFoot_RF", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  mpcPolygonMsgSubscriber_[2] = nodeHandle.subscribe("foothold_planner/RegionForFoot_LH", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
  mpcPolygonMsgSubscriber_[3] = nodeHandle.subscribe("foothold_planner/RegionForFoot_RH", 1,
        &LegEndEffectorsPolygonReceiver::mpcPolygonMsgCallback, this, ::ros::TransportHints().udp());
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
  // std::cout << "msg size:" << msg->region.size() << " " << msg->region[0].boundaryPoint.size() << "\n";
  for (int i = 0; i < msg->region.size(); i++) {
      std::vector<Eigen::Vector3d> polygon;
      polygon.reserve(msg->region[i].boundaryPoint.size());
      for (int j = 0; j < msg->region[i].boundaryPoint.size(); j++) {
          polygon.push_back(Eigen::Vector3d(msg->region[i].boundaryPoint[j].x, msg->region[i].boundaryPoint[j].y, msg->region[i].boundaryPoint[j].z));
      }
      receivedFeetPoints_[foot_id].push_back(polygon);
  }

  polygonsUpdated_ = true;
}

void LegEndEffectorsPolygonReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                            const ReferenceManagerInterface& referenceManager) {
  if(polygonsUpdated_){
    std::lock_guard<std::mutex> lock(receivedPolygonMsgMutex_);
    std::cout << "polygonsUpdated_\n";
    // point transformed to the world frame. 4x4 tf matrix.
    const auto& currentPose = initState.segment<6>(6);
    matrix_t tfMatrix = matrix_t::Identity(4,4);
    const vector3_t ZyxEulerAngles = currentPose.tail(3);
    const matrix3_t rotationMatrix = ocs2::getRotationMatrixFromZyxEulerAngles(ZyxEulerAngles);
    tfMatrix.topLeftCorner(3,3) = rotationMatrix;
    tfMatrix.topRightCorner(3,1) = currentPose.head(3);

    transformedFeetPoints_ = receivedFeetPoints_; // this is deep copy.

    // transformedPolygons_

    size_t leg = 0, polygonIndex = 0, pointIndex = 0;
    for(const auto& polygons:receivedFeetPoints_){
      // transformedPolygons_[leg].clear();
      // transformedPolygons_[leg].reserve(polygons.size());
      polygonIndex = 0;
      for(const auto& polygon:polygons){
        // transformedPolygons_[leg][polygonIndex].reserve(polygon.size());
        std::vector<ocs2::Polygon::Position> xyPoints;
        xyPoints.reserve(polygon.size());
        pointIndex = 0;
        for(const auto& point:polygon){
          const auto& transformedPoint = tfMatrix * point.homogeneous();
          transformedFeetPoints_[leg][polygonIndex][pointIndex] = transformedPoint.matrix().topRows(3);
          xyPoints.push_back(transformedPoint.matrix().topRows(2));
          ++pointIndex;
        }
        // transformedPolygons_[leg].emplace_back(xyPoints);
        ++polygonIndex;
      }
      ++leg;
    }

    *mpcTransformedPolygonsPtr_ = transformedFeetPoints_;

    // save the transformed points to let footplacementplanner chose.
    // create polygon using transformed points.
    // form the constraints. (cteate constraints struct)

    // legEndeffectorPolygonReceived_->copy(terrainEstDataTemp_);
    polygonsUpdated_ = false;
  }

}

LegEndEffectorsPolygonReceiver::~LegEndEffectorsPolygonReceiver(){}

} // namespace legged_robot
} // namespace ocs2

