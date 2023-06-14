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

#include "ocs2_jypro/foot_planner/QuinticSpline.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuinticSpline::QuinticSpline(Node start, Node middle, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;
  
  scalar_t x1 = start.position;
  scalar_t x2 = middle.position;
  scalar_t x3 = end.position;

  scalar_t v1 = start.velocity * dt_;
  scalar_t v2 = middle.velocity * dt_;
  scalar_t v3 = end.velocity * dt_;

  c0_ = x1;
  c1_ = v1;
  c2_ = 16 * x2 - 8 * v2 - v3 - 23 * x1 - 6 * v1 + 7 * x3;
  c3_ = 13 * v1 + 32 * v2 + 5 * v3 + 66 * x1 - 32 * x2 - 34 * x3;
  c4_ = 16 * x2 - 40 * v2 - 8 * v3 - 68* x1 - 12 * v1 + 52 * x3;
  c5_ = 4 * v1 + 16 * v2 + 4 * v3 + 24 * x1 - 24 * x3; 

}

QuinticSpline::QuinticSpline(Node start, scalar_t middle, Node end, bool isLeft) {
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
  if(isLeft) {
    c0_ = x1;
    c1_ = v1;
    c2_ = 5*v3 -5*v1 -a3/2 -16*x1 +32*x2 -16*x3;
    c3_ = 2*a3 +9*v1 -19*v3 + 38*x1 -96*x2 +58*x3;
    c4_ = 22*v3 - 7*v1 - 2.5*a3 - 33*x1 + 96*x2 - 63*x3;
    c5_ = a3 +2*v1 -8*v3 +10*x1 -32*x2 +22*x3; 
  }
  else {
    c0_ = x1;
    c1_ = v1;
    c2_ = a1/2;
    c3_ = v3 -11*v1 -2*a1 -26*x1 +32*x2 -6*x3;
    c4_ = 2.5*a1 +18*v1 -3*v3 +47*x1 -64*x2 +17*x3;
    c5_ = 2*v3 -8*v1 -a1 -22*x1 +32*x2 -10*x3;
  }

}




/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuinticSpline::position(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return c5_ * tn * tn * tn * tn * tn  + c4_ * tn * tn * tn * tn + c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuinticSpline::velocity(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (5.0 * c5_ * tn * tn * tn * tn + 4.0 * c4_ * tn * tn * tn + 3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuinticSpline::acceleration(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (20.0 * c5_ * tn * tn * tn + 12.0 * c4_ * tn * tn + 6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuinticSpline::normalizedTime(scalar_t t) const {
  assert(t >= t0_);
  assert(t <= t1_);
  return (t - t0_) / dt_;
}
}  // namespace legged_robot
}  // namespace ocs2
