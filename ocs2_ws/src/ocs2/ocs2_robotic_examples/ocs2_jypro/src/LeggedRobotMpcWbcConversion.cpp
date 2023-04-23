#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <mutex>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/LeggedRobotInterface.h"
#include <ocs2_core/loopshaping/Loopshaping.h>

// MPC OUTPUT FOR UDP
#define SEND_PORT 1111
#define SEND_IP "127.0.0.1"

#define LENGTH 10
#define LENGTH 10
// size_t N_times = LENGTH;
using vector_foot_t = Eigen::Matrix<Eigen::Matrix<Eigen::Matrix<float, 3, 1 >,4, 1>, LENGTH, 1>;
using vector_base_t = Eigen::Matrix <Eigen::Matrix<float, 6, 1>, LENGTH, 1>;
using vector_joint_t = Eigen::Matrix <Eigen::Matrix<float, 12, 1>, LENGTH, 1>;
struct conversionData{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    vector_foot_t swingFeetPosition; //lf lh rf rh
    vector_foot_t swingFeetVelocity; //lf lh rf rh
    vector_foot_t swingFeetAcceleration;  //lf lh rf rh
    Eigen::Vector4f firstGait; //lf lh rf rh
    Eigen::Vector4f secondGait; //lf lh rf rh
    Eigen::Vector4f thirdGait; //lf lh rf rh
    Eigen::Vector2f switchTime;
    vector_base_t basePosition;
    vector_base_t baseVelocity;
    vector_base_t baseAcceleration;
    vector_joint_t jointPos;
    vector_joint_t jointVel;
    vector_joint_t jointAcc;
    Eigen::Matrix<float, LENGTH,1> stateTime;
    Eigen::Matrix<float, 12, 1> inputForce;
};

namespace ocs2 {
namespace legged_robot {

class LeggedRobotMpcWbcCobversion : public SolverSynchronizedModule {
 public:
  LeggedRobotMpcWbcCobversion(const std::string& robotName, LeggedRobotInterface& interface,
                              std::shared_ptr<LoopshapingDefinition> loopShapingDefinition);
  
  ~LeggedRobotMpcWbcCobversion(){ close(send_fd); }

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override {};

  void postSolverRun(const PrimalSolution& primalSolution) override;
 
 private:
  
  int send_fd;
  struct sockaddr_in send_aadr;

  PinocchioInterface& pinocchioInterface_;
  std::shared_ptr<LoopshapingDefinition> loopShapingDefinitionPtr_;
  bool usingLoopshaping_;

  conversionData wbcInterfaceData;
  

};

LeggedRobotMpcWbcCobversion::LeggedRobotMpcWbcCobversion(const std::string& robotName, 
 LeggedRobotInterface& interface, std::shared_ptr<LoopshapingDefinition> loopShapingDefinitionPtr) : 
  pinocchioInterface_(interface.getPinocchioInterface()),
  loopShapingDefinitionPtr_(std::move(loopShapingDefinitionPtr)){
    usingLoopshaping_ = (loopShapingDefinitionPtr_ != nullptr);

    send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(send_fd < 0){
        printf("create socket fail!\n");
        throw std::runtime_error("[LeggedRobotMpcWbcCobversion] create socket failed !");
    }
    memset(&send_aadr, 0 , sizeof(send_aadr));
    send_aadr.sin_family = AF_INET;
    send_aadr.sin_addr.s_addr = inet_addr(SEND_IP);
    send_aadr.sin_port = htons(SEND_PORT);
}

void LeggedRobotMpcWbcCobversion::postSolverRun(const PrimalSolution& primalSolution){
  const auto& timeTrajectory = primalSolution.timeTrajectory_;
  const auto& stateTrajectory = primalSolution.stateTrajectory_;
  const auto& inputTrajectory = primalSolution.inputTrajectory_;
  const size_t N = timeTrajectory.size();

  if(usingLoopshaping_){
    PrimalSolution originalSolution(primalSolution);
    originalSolution.stateTrajectory_.reserve(N);
    originalSolution.inputTrajectory_.reserve(N);
    for(size_t i = 0; i < N; i++) {
      originalSolution.stateTrajectory_[i] = 
        loopShapingDefinitionPtr_->getSystemState(stateTrajectory[i]);
      originalSolution.inputTrajectory_[i] =
        loopShapingDefinitionPtr_->getSystemInput(stateTrajectory[i], inputTrajectory[i]);
    }
  }

}

} //namespace ocs2
} //namespace legged_robot