/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_jypro/foot_planner/TwoQuinticSplineCpg.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TwoQuinticSplineCpg::TwoQuinticSplineCpg(QuinticSpline::Node liftOff, QuinticSpline::Node middleLeft, scalar_t midHeight, scalar_t midTime,
                          QuinticSpline::Node middleRight, QuinticSpline::Node touchDown)
    : midTime_(midTime),
      leftSpline_(liftOff, middleLeft, QuinticSpline::Node{midTime_, midHeight, 0.0}),
      rightSpline_(QuinticSpline::Node{midTime_, midHeight, 0.0}, middleRight, touchDown) {}

TwoQuinticSplineCpg::TwoQuinticSplineCpg(QuinticSpline::Node liftOff, scalar_t leftMidHeight, QuinticSpline::Node apex, 
      scalar_t rightMidHeight,QuinticSpline::Node touchDown) 
    : midTime_(apex.time),
      leftSpline_(liftOff, leftMidHeight, apex, true),
      rightSpline_(apex, rightMidHeight, touchDown, false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TwoQuinticSplineCpg::position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TwoQuinticSplineCpg::velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TwoQuinticSplineCpg::acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}

}  // namespace legged_robot
}  // namespace ocs2
