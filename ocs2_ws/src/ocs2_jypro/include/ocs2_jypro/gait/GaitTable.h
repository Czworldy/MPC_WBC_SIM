#pragma once

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>
#include "ocs2_jypro/gait/ModeSequenceTemplate.h"
#include "ocs2_jypro/gait/MotionPhaseDefinition.h"
#include <ocs2_core/misc/Display.h>


namespace ocs2 {
namespace legged_robot {

struct GaitTable
{
  feet_array_t<std::vector<bool>> isContact;
  feet_array_t<std::vector<scalar_t>> contactSwitchTimes;
  
};

std::ostream& operator<<(std::ostream& stream, const GaitTable& gaitTable) {
  stream << "GaitTable:       " << "\n";
  for (size_t i = 0; i < 4; i++) {
    stream << "Event phases:  {" << toDelimitedString(gaitTable.isContact[i]) << "}\n";
    stream << "Mode sequence: {" << toDelimitedString(gaitTable.contactSwitchTimes[i]) << "}\n";
  }

  return stream;
}

GaitTable getGaitTable(const ModeSequenceTemplate& modeSequenceTemplate){
  const auto& modeSequence = modeSequenceTemplate.modeSequence;
  const auto& eventTimes = modeSequenceTemplate.switchingTimes;
  // std::cout << "modeSequenceTemplate: " << modeSequenceTemplate << std::endl;
  GaitTable gaitTable;
  feet_array_t<std::vector<bool>> isContact;
  feet_array_t<std::vector<scalar_t>> contactSwitchTimes;

  contactSwitchTimes.fill(eventTimes);
  for(const auto& mode : modeSequence){
    auto contactFlags = modeNumber2StanceLeg(mode);
    for(size_t i = 0; i < contactFlags.size(); i++){
      isContact[i].push_back(contactFlags[i]);
    }
  }

  for (size_t i = 0; i < 4; i++){
    gaitTable.isContact[i].push_back(isContact[i][0]);
    gaitTable.contactSwitchTimes[i].push_back(contactSwitchTimes[i][0]);
    for (size_t j = 0; j < isContact[i].size() -1; j++){
      if (isContact[i][j] != isContact[i][j+1]){
        gaitTable.isContact[i].push_back(isContact[i][j+1]);
        gaitTable.contactSwitchTimes[i].push_back(contactSwitchTimes[i][j+1]);
      }
    }
    gaitTable.contactSwitchTimes[i].push_back(contactSwitchTimes[i].back());
  }
  
  return gaitTable;
}

GaitTable getGaitTable(const ModeSchedule& modeSchedule){
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;
  // std::cout << "modeSchedule: " << modeSchedule << std::endl;
  GaitTable gaitTable;
  feet_array_t<std::vector<bool>> isContact;
  feet_array_t<std::vector<scalar_t>> contactSwitchTimes;

  contactSwitchTimes.fill(eventTimes);
  for(const auto& mode : modeSequence){
    auto contactFlags = modeNumber2StanceLeg(mode);
    for(size_t i = 0; i < contactFlags.size(); i++){
      isContact[i].push_back(contactFlags[i]);
    }
  }

  for (size_t i = 0; i < 4; i++){
    gaitTable.isContact[i].push_back(isContact[i][0]);
    // gaitTable.contactSwitchTimes[i].push_back(contactSwitchTimes[i][0]);
    for (size_t j = 0; j < isContact[i].size() -1; j++){
      if (isContact[i][j] != isContact[i][j+1]){
        gaitTable.isContact[i].push_back(isContact[i][j+1]);
        gaitTable.contactSwitchTimes[i].push_back(contactSwitchTimes[i][j]);
      }
    }
    // gaitTable.contactSwitchTimes[i].push_back(contactSwitchTimes[i].back());
  }
  
  return gaitTable;
}

bool isValidTime(const GaitTable& gaitTable, const scalar_t& time) {
  return time >= 0.0 && time <= gaitTable.contactSwitchTimes[0].back();
}

feet_array_t<scalar_t> timeLeftCurrentPhase(const GaitTable& gaitTable, const scalar_t& time){
  // isValidTime(gaitTable, time);
  // scalar_t time_ = time;
  // while (time_ > gaitTable.contactSwitchTimes[0].back()){
  //   time_ -= gaitTable.contactSwitchTimes[0].back();
  // }
  // std::cout << "current table: " << gaitTable << std::endl;
  feet_array_t<scalar_t> timeLeftCurrentPhase;
  timeLeftCurrentPhase.fill(0.0);
  if(gaitTable.contactSwitchTimes[0].empty()
      || gaitTable.contactSwitchTimes[1].empty()
        || gaitTable.contactSwitchTimes[2].empty()
          || gaitTable.contactSwitchTimes[3].empty()){
    return timeLeftCurrentPhase;
  }
  for (size_t i = 0; i < 4; i++){
    for (size_t j = 0; j < gaitTable.contactSwitchTimes[i].size() - 1; j++){
      
      if (time >= gaitTable.contactSwitchTimes[i][j] && time < gaitTable.contactSwitchTimes[i][j+1]){
        timeLeftCurrentPhase[i] = gaitTable.contactSwitchTimes[i][j+1] - time;
      }
      else if(time < gaitTable.contactSwitchTimes[i].front()){
        timeLeftCurrentPhase[i] =  gaitTable.contactSwitchTimes[i].front() - time;
      }
    }
  }

  return timeLeftCurrentPhase;
}

feet_array_t<bool> getContactState(const GaitTable& gaitTable, const scalar_t& time){
  // isValidTime(gaitTable, time);
  // scalar_t time_ = time;
  // while (time_ > gaitTable.contactSwitchTimes[0].back()){
  //   time_ -= gaitTable.contactSwitchTimes[0].back();
  // }
  feet_array_t<bool> contactState;
  contactState.fill(true);
  if(gaitTable.contactSwitchTimes[0].empty()
      || gaitTable.contactSwitchTimes[1].empty()
        || gaitTable.contactSwitchTimes[2].empty()
          || gaitTable.contactSwitchTimes[3].empty()){
    return contactState;
  }
  for (size_t i = 0; i < 4; i++){
    for (size_t j = 0; j < gaitTable.contactSwitchTimes[i].size() - 1; j++){
      if (time >= gaitTable.contactSwitchTimes[i][j] && time < gaitTable.contactSwitchTimes[i][j+1]){
        contactState[i] = gaitTable.isContact[i][j+1];
      }
    }
  }

  return contactState;
}



}  // namespace legged_robot
}  // namespace ocs2

