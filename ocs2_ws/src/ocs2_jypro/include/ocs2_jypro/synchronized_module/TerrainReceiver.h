
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ros/ros.h>
#include <mutex>

#include <ocs2_msgs/mpc_terrain.h>
#include "ocs2_jypro/common/Types.h"


namespace ocs2 {
namespace legged_robot {

struct TerrainEstData{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
  TerrainEstData(){
    terrainQuat.setIdentity();
    terrainParams.setZero();
    feetHeight.setZero();
  }

  TerrainEstData(const Eigen::Quaternionf& quat, const Eigen::Vector3f& params, const Eigen::Vector4f& height, const contact_flag_t& stanceLegs){
    this->terrainQuat = quat;
    this->terrainParams = params;
    this->feetHeight = height;
    this->stanceLegs = stanceLegs;
  }

  void copy(const TerrainEstData& other){
    terrainQuat = other.terrainQuat;
    terrainParams = other.terrainParams;
    feetHeight = other.feetHeight;
    stanceLegs = other.stanceLegs;
  }

  Eigen::Quaternionf terrainQuat;
  Eigen::Vector3f terrainParams;
  Eigen::Vector4f feetHeight; //lf lh rf rh 
  contact_flag_t stanceLegs;  // {LF, RF, LH, RH}
};

std::ostream& operator<<(std::ostream& stream, const TerrainEstData& terrainEstData);

class TerrainReceiver : public SolverSynchronizedModule {
 public:
  TerrainReceiver(ros::NodeHandle& nodeHandle, std::shared_ptr<TerrainEstData> terrainEstDataPtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

  const std::shared_ptr<TerrainEstData> getTerrainEstDataPtr() const { return terrainEstDataPtr_; }
  
 private:
  void mpcTerrainMsgCallback(const ocs2_msgs::mpc_terrain::ConstPtr& msg);

  ros::Subscriber mpcTerrainMsgSubscriber_;

  std::mutex receivedTerrainMsgMutex_;
  std::atomic_bool terrainUpdated_;

  TerrainEstData terrainEstDataTemp_;
  std::shared_ptr<TerrainEstData> terrainEstDataPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2