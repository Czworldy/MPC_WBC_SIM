#include "ocs2_jypro/synchronized_module/TerrainPythonInterface.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TerrainPythonInterface::TerrainPythonInterface(std::shared_ptr<TerrainEstData> terrainEstDataPtr) 
    : terrainEstDataPtr_(std::move(terrainEstDataPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainPythonInterface::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (terrainUpdated_) {
    std::lock_guard<std::mutex> lock(receivedTerrainMsgMutex_);
    // std::cerr << "[TerrainPythonInterface]: Setting new terrain after time " << finalTime << "\n";
    // std::cerr << terrainEstDataTemp_;
    terrainEstDataPtr_->copy(terrainEstDataTemp_);
    terrainUpdated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainPythonInterface::setMpcTerrain(const TerrainEstData& msg) {
  std::lock_guard<std::mutex> lock(receivedTerrainMsgMutex_);
  terrainEstDataTemp_ = msg;
  terrainUpdated_ = true;
}

}  // namespace legged_robot
}  // namespace ocs2

