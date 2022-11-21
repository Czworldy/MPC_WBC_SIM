#pragma once

#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "Types.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Counts contact feet */
inline size_t numberOfClosedContactFeet(const contact_flag_t& contactFlags) {
    size_t numContactLegs = 0;
    for (int legInContact = 0; legInContact < 4; legInContact++) { // just legs
        if(contactFlags[legInContact]) {
            ++numContactLegs;
        }      
    }
    return numContactLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact feet. */
inline vector_t weightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info, const contact_flag_t& contactFlags) {
  const auto numStanceLegs = numberOfClosedContactFeet(contactFlags);
  vector_t input = vector_t::Zero(info.inputDim);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = info.robotMass * 9.81;
    const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {
      if (contactFlags[i]) {
        centroidal_model::getContactForces(input, i, info) = forceInInertialFrame;
      }
    }  // end of i loop
  }
  return input;
}

} // namespace legged_robot
} // namespace ocs2