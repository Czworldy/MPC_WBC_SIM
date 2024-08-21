#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_jypro/common/Types.h"
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include "ocs2_jypro/constraint/Polygon.hpp"
#include "ocs2_jypro/synchronized_module/LegEndEffectorsPolygonReceiver.h"


namespace ocs2 {
namespace legged_robot {

struct FootConstraints{
  matrix_t A;
  vector_t b;
};

class FootConstraintsPlanner{
 public:
 using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;



   FootConstraintsPlanner(PinocchioInterface& pinocchioInterface,
                         const PinocchioEndEffectorKinematics& endEffectorKinematics,
                         const CentroidalModelInfo& centroidalModelInfo,
                         size_t numFeet);

   void update(const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories,
                    scalar_t initTime, const vector_t& initState);
   vector3_t getCurrentEEPosition(size_t leg, const vector_t& initstate);

   vector3_t getFootPlacementNominal(size_t leg,  scalar_t time) const;

   void setMpcTrajectoryAccordingToFootPlacement(const scalar_t initTime, const ModeSchedule& modeSchedule,
                                                 TargetTrajectories& targetTrajectories, scalar_t comHeight);

   const FootConstraints& getFootPolygonConstraint(size_t leg,  scalar_t time) const;

   const feet_array_t<scalar_array_t>& getliftOffHeightSequence() const{ return liftOffHeightSequence_; }
   const feet_array_t<scalar_array_t>& gettouchDownHeightSequence() const{ return touchDownHeightSequence_; }
   const feet_array_t<scalar_array_t>& getfeetPlacementEvents() const{ return feetPlacementEvents_; }
   const feet_array_t<std::vector<vector3_t>>& getfeetPlacement() const{ return feetPlacement_; }

   void setTargetPolygonVerteices(const feet_polygon_array_t& legEndEffectorPolygon, 
                            const feet_array_t<std::vector<vector3_t>>& nominalFoothold ){
    // std::cout << "legEndEffectorPolygon set size:" << legEndEffectorPolygon.size() << "\n";
    legEndEffectorPolygon_ = legEndEffectorPolygon;
    nominalFoothold_ = nominalFoothold;
   }

   void setTargetSwingHeight(const feet_array_t<vector_array_t>& mpcSwingHeight){
    swingHeight_ = mpcSwingHeight;
   }

    void setTargetSwingMiddleTime(const feet_array_t<scalar_array_t>& mpcSwingMiddleTime){
    swingMiddleTime_ = mpcSwingMiddleTime;
   }

   const feet_array_t<std::vector<FootConstraints>>& getFootConstraints() const{ return feetPlacementConstraints_; }

   const feet_array_t<std::vector<vector3_t>>& getNominalFoothold() const{ return nominalFoothold_; }

   const feet_array_t<vector_array_t>& getSwingHeightSequence() const{ return swingHeightSequence_; }

   const feet_array_t<std::vector<scalar_t>>& getSwingMiddleTimeSequence() const{ return swingMiddleTimeSequence_; }

 private:
      /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
    feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;
    feet_array_t<std::vector<bool>> extractSwingFlags(const std::vector<size_t>& phaseIDsStock) const;

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

  //  vector3_t choiceCloestFootPlacement(const size_t& footNum, const vector3_t& position);
   std::pair<int, vector3_t> choiceCloestPolygonVertex(const size_t& footNum, const vector3_t& position);

   PinocchioInterface& pinocchioInterface_;
   std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;
   const CentroidalModelInfo& centroidalModelInfo_;

   const size_t numFeet_;
  //  std::vector<vector3_t> leftFrontPoints_;
  //  std::vector<vector3_t> rightFrontPoints_;
  //  std::vector<vector3_t> leftBackPoints_;
  //  std::vector<vector3_t> rightBackPoints_;

   feet_polygon_array_t legEndEffectorPolygon_;
   feet_array_t<std::vector<vector3_t>> nominalFoothold_;
   feet_array_t<vector_array_t> swingHeight_;
   feet_array_t<std::vector<scalar_t>> swingMiddleTime_;

   feet_array_t<std::vector<vector3_t>> feetPlacement_;
   feet_array_t<vector_array_t> swingHeightSequence_;
   feet_array_t<std::vector<scalar_t>> swingMiddleTimeSequence_;
   feet_array_t<std::vector<scalar_t>> feetPlacementEvents_;
   feet_array_t<std::vector<FootConstraints>> feetPlacementConstraints_;

   feet_array_t<scalar_array_t> liftOffHeightSequence_;
   feet_array_t<scalar_array_t> touchDownHeightSequence_;

};

}
}