// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/foot_planner/FootPlacementPlanner.h"
#include "ocs2_jypro/gait/MotionPhaseDefinition.h"

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
// #include <ocs2_centroidal_model/CentroidalModelInfo.h>



namespace ocs2 {
namespace legged_robot {


FootPlacementPlanner::FootPlacementPlanner(PinocchioInterface& pinocchioInterface, 
                                           const PinocchioEndEffectorKinematics& endEffectorKinematics,
                                           const CentroidalModelInfo& centroidalModelInfo,
                                           size_t numFeet)
  : pinocchioInterface_(pinocchioInterface),
    endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
    centroidalModelInfo_(centroidalModelInfo),
    numFeet_(numFeet) {
      endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);

      for(size_t i = 0; i < 10; ++i) {
        Eigen::Matrix<scalar_t, 3, 1> leftpoint = {-0.177, 0.0, 0.0};
        Eigen::Matrix<scalar_t, 3, 1> rightpoint = {0.177, 0.0, 0.0};
        if(i < 3){
          leftpoint[1] = 0.25*i - 0.388;
          rightpoint[1] = 0.25*i - 0.388;
        }
        else{
          leftpoint[1] = 0.25*(i - 3) + 0.388;
          rightpoint[1] = 0.25*(i - 3) + 0.388;
        }
        leftPoints.emplace_back(leftpoint);
        rightPoints.emplace_back(rightpoint);
      }
    }

vector3_t FootPlacementPlanner::getFootPlacementConstraint(size_t leg,  scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetPlacementEvents_[leg], time);
  return feetPlacement_[leg][index];
}

void FootPlacementPlanner::update(const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories, const scalar_t & initTime){
  const auto& modeSequence_ = modeSchedule.modeSequence;
  const auto& eventTimes_ = modeSchedule.eventTimes;

  size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

  // cut those past sequence
  std::vector<size_t> modeSequence(modeSequence_.begin() + initIndex, modeSequence_.end());
  std::vector<scalar_t> eventTimes(eventTimes_.begin() + initIndex, eventTimes_.end());  

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    if (eesContactFlagStocks[j][0]) { // currently stance leg
      feetPlacement_[j].clear();
      feetPlacement_[j].reserve(modeSequence.size());
      feetPlacementEvents_[j] = eventTimes;
      for (int p = 1; p < modeSequence.size(); ++p) {
        if (!eesContactFlagStocks[j][p]) { // for next sqing phases 
          const int swingStartIndex = startTimesIndices[j][p];
          const int swingFinalIndex = finalTimesIndices[j][p];
          checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

          const scalar_t swingStartTime = eventTimes[swingStartIndex];
          const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

          const vector_t desiredstate = targetTrajectories.getDesiredState(swingFinalTime);

          // std::cout << "swingFinalTime: " << swingFinalTime << std::endl;

          const auto& model = pinocchioInterface_.getModel();
          auto& data = pinocchioInterface_.getData();
          pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(desiredstate, centroidalModelInfo_));
          pinocchio::updateFramePlacements(model, data);

          const auto feetPosition = endEffectorKinematicsPtr_->getPosition(desiredstate)[j];

          // std::cout << "footpos: " << feetPosition.transpose() << std::endl;

          vector3_t footplacement = choiceCloestFootPlacement(j, feetPosition);
          feetPlacement_[j].emplace_back(footplacement);          
        }
        else{// for a stance leg
          feetPlacement_[j].emplace_back(0,0,0);
        }
      }
    }
  }


  // int i = 0;
  // for(auto leg:feetPlacement_){
  //   std::cout << "leg:" << i << "===================" << std::endl;
  //   for(auto foot:leg){
  //     std::cout << "point" <<foot.transpose() << std::endl;
  //   }
  //   i++;
  // }
} 

vector3_t FootPlacementPlanner::choiceCloestFootPlacement(const size_t& footNum, const vector3_t& position){
  scalar_t minDistance = 100;
  vector3_t minPoint;

  if(footNum == 0||footNum == 2){// for left feet
    for(const auto& leftpoint:leftPoints){
      scalar_t distance = (leftpoint - position).norm();
      if(distance < minDistance){
        minDistance = distance;
        minPoint = leftpoint;
      }
    }
  }
  else{// for right feet
    for(const auto& rightpoint:rightPoints){
      scalar_t distance = (rightpoint - position).norm();
      if(distance < minDistance){
        minDistance = distance;
        minPoint = rightpoint;
      }
    }
  }

  return minPoint;
}

void FootPlacementPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock) {
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
}



feet_array_t<std::vector<bool>> FootPlacementPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < numFeet_; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

std::pair<std::vector<int>, std::vector<int>> FootPlacementPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing feet
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

std::pair<int, int> FootPlacementPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index]) {
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}


} // namespace legged_robot
} // namespace ocs2
