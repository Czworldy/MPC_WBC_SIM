/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_core/reference/TargetTrajectories.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
TargetTrajectories::TargetTrajectories(size_t size) : timeTrajectory(size), stateTrajectory(size), inputTrajectory(size), eePositionTrajectory(size) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
TargetTrajectories::TargetTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                                       vector_array_t desiredInputTrajectory, vector_array_t desiredEEPositionTrajectory)
    : timeTrajectory(std::move(desiredTimeTrajectory)),
      stateTrajectory(std::move(desiredStateTrajectory)),
      inputTrajectory(std::move(desiredInputTrajectory)),
      eePositionTrajectory(std::move(desiredEEPositionTrajectory)) {
  assert(stateTrajectory.size() == timeTrajectory.size());
  if (!inputTrajectory.empty()) {
    assert(inputTrajectory.size() == timeTrajectory.size());
  }
  if (!eePositionTrajectory.empty()) {
    assert(eePositionTrajectory.size() == timeTrajectory.size());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void TargetTrajectories::clear() {
  timeTrajectory.clear();
  stateTrajectory.clear();
  inputTrajectory.clear();
  eePositionTrajectory.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
bool TargetTrajectories::operator==(const TargetTrajectories& other) {
  return this->timeTrajectory == other.timeTrajectory && this->stateTrajectory == other.stateTrajectory &&
         this->inputTrajectory == other.inputTrajectory && this->eePositionTrajectory == other.eePositionTrajectory;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t TargetTrajectories::getDesiredState(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories is empty!");
  } else {
    return LinearInterpolation::interpolate(time, timeTrajectory, stateTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t TargetTrajectories::getDesiredInput(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories is empty!");
  } else if (inputTrajectory.empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories does not have inputTrajectory!");
  } else {
    return LinearInterpolation::interpolate(time, timeTrajectory, inputTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t TargetTrajectories::getDesiredEEPosition(scalar_t time) const {
  if(this->empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories is empty!");
  } else if (eePositionTrajectory.empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories does not have eePositionTrajectory!");
  } else {
    return LinearInterpolation::interpolate(time, timeTrajectory, eePositionTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void swap(TargetTrajectories& lh, TargetTrajectories& rh) {
  lh.timeTrajectory.swap(rh.timeTrajectory);
  lh.stateTrajectory.swap(rh.stateTrajectory);
  lh.inputTrajectory.swap(rh.inputTrajectory);
  lh.eePositionTrajectory.swap(rh.eePositionTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
std::ostream& operator<<(std::ostream& out, const TargetTrajectories& targetTrajectories) {
  for (size_t i = 0; i < targetTrajectories.size(); i++) {
    out << "time: " << targetTrajectories.timeTrajectory[i] << "\n";
    out << "state: [" << toDelimitedString(targetTrajectories.stateTrajectory[i]) << "]\n";
    out << "input: [" << toDelimitedString(targetTrajectories.inputTrajectory[i]) << "]\n";
    out << "eePosition: [" << toDelimitedString(targetTrajectories.eePositionTrajectory[i]) << "]\n";
  }  // end of i loop

  return out;
}

}  // namespace ocs2
