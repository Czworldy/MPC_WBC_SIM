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
    FootPlacementPlanner(PinocchioInterface pinocchioInterface, 
                         const PinocchioEndEffectorKinematics& endEffectorKinematics,
                         const CentroidalModelInfo& centroidalModelInfo,
                         size_t numFeet);
    ~FootPlacementPlanner();

    void update(const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories, const scalar_t& initTime);
 private:
      /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
    feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
    static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
    static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);
    void checkThatIndicesAreValid(int leg, int index, int startIndex, 
                                        int finalIndex, const std::vector<size_t>& phaseIDsStock);
    
    PinocchioInterface pinocchioInterface_;
    std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;
    const CentroidalModelInfo& centroidalModelInfo_;

    const size_t numFeet_;


};

}
}