#include "ocs2_jypro/synchronized_module/TerrainReceiver.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TerrainReceiver::TerrainReceiver(ros::NodeHandle& nodeHandle, std::shared_ptr<TerrainEstData> terrainEstDataPtr, const std::string& robotName) 
    : terrainEstDataPtr_(std::move(terrainEstDataPtr)) {
  mpcTerrainMsgSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_terrain", 1, 
      &TerrainReceiver::mpcTerrainMsgCallback, this, ::ros::TransportHints().udp());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (terrainUpdated_) {
    std::lock_guard<std::mutex> lock(receivedTerrainMsgMutex_);
    // std::cerr << "[TerrainReceiver]: Setting new terrain after time " << finalTime << "\n";
    // std::cerr << terrainEstDataTemp_;
    terrainEstDataPtr_->copy(terrainEstDataTemp_);
    terrainUpdated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainReceiver::mpcTerrainMsgCallback(const ocs2_msgs::mpc_terrain::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(receivedTerrainMsgMutex_);

  Eigen::Quaternionf quat = Eigen::Quaternionf(msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z);
  Eigen::Vector3f params = {msg->a, msg->b, msg->d};
  Eigen::Vector4f heights = {msg->feetHeight[0], msg->feetHeight[1], msg->feetHeight[2], msg->feetHeight[3]};
  contact_flag_t stanceLegs;

  // copy msg->contace to stanceLegs
  for(int i = 0; i < 4; i++){
    stanceLegs[i] = (bool)msg->contact[i];
  }

  terrainEstDataTemp_ = TerrainEstData(quat, params, heights, stanceLegs);
  terrainUpdated_ = true;
}

std::ostream& operator<<(std::ostream& stream, const TerrainEstData& terrainEstData){
  stream << "TerrainEstData: " << "\n";
  stream << "terrainQuat: " << terrainEstData.terrainQuat.coeffs().transpose() << "\n";
  stream << "terrainParams: " << terrainEstData.terrainParams.transpose() << "\n";
  stream << "feetHeight lf lh rf rh: " << terrainEstData.feetHeight.transpose() << "\n stanceLegs:";
  for(int i = 0; i < 4; i++){
    stream << " " << (bool)terrainEstData.stanceLegs[i];
  }
  stream << std::endl;
  return stream;
}

}  // namespace legged_robot
}  // namespace ocs2