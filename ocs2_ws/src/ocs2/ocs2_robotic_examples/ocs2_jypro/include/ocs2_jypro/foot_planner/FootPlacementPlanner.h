#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_jypro/common/Types.h"
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>




namespace ocs2 {
namespace legged_robot {

class FootPlacementPlanner{
 public:
    FootPlacementPlanner(/* args */);
    ~FootPlacementPlanner();

    void update(const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories);
 private:
    const PinocchioInterface& pinocchioInterface_;
    std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;
    const CentroidalModelInfo& centroidalModelInfo_;


};

}
}