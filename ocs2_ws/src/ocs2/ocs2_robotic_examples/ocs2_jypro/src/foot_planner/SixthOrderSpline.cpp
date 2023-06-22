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

#include "ocs2_jypro/foot_planner/SixthOrderSpline.h"

namespace ocs2 {
namespace legged_robot {



SixthOrderSpline::SixthOrderSpline(Node start, scalar_t middle, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;
  
  scalar_t x1 = start.position;
  scalar_t x2 = middle;
  scalar_t x3 = end.position;

  scalar_t v1 = start.velocity * dt_;
  scalar_t v3 = end.velocity * dt_;
  
  scalar_t a3 = end.acceleration * dt_ * dt_;
  scalar_t a1 = start.acceleration * dt_ * dt_;

  c0_ = x1;
  c1_ = v1;
  c2_ = a1/2;
  c3_ = 64*x2 -a3/2 -2.5*a1 - 22*x3 -16*v1 +6*v3 -42*x1;
  c4_ = 4.5*a1+2*a3 - 192*x2 +81*x3 +38*v1 - 23*v3+ 111*x1;
  c5_ = 192*x2 - 2.5*a3- 3.5*a1 - 90*x3 - 33*v1 + 27*v3 - 102*x1; 
  c6_ = a1 +a3 - 64*x2 + 32*x3 + 10*v1 -10*v3 +32*x1;


}




/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SixthOrderSpline::position(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return c6_* tn * tn * tn * tn * tn * tn + c5_ * tn * tn * tn * tn * tn  + c4_ * tn * tn * tn * tn + c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SixthOrderSpline::velocity(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (6*c6_* tn * tn * tn * tn * tn + 5.0 * c5_ * tn * tn * tn * tn + 4.0 * c4_ * tn * tn * tn + 3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SixthOrderSpline::acceleration(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (30 * c6_* tn * tn * tn * tn + 20.0 * c5_ * tn * tn * tn + 12.0 * c4_ * tn * tn + 6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SixthOrderSpline::normalizedTime(scalar_t t) const {
  assert(t >= t0_);
  assert(t <= t1_);
  return (t - t0_) / dt_;
}


TwoSixthOrderSplineCpg::TwoSixthOrderSplineCpg(Node liftOff, scalar_t leftMidHeight, Node apex, 
      scalar_t rightMidHeight,Node touchDown) 
    : midTime_(apex.time),
      leftSpline_(liftOff, leftMidHeight, apex),
      rightSpline_(apex, rightMidHeight, touchDown) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TwoSixthOrderSplineCpg::position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TwoSixthOrderSplineCpg::velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TwoSixthOrderSplineCpg::acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}
}  // namespace legged_robot
}  // namespace ocs2
