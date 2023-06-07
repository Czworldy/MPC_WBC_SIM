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

#pragma once

#include <ocs2_core/Types.h>
#include "ocs2_jypro/foot_planner/CubicSpline.h"
#include "ocs2_jypro/foot_planner/QuinticSpline.h"
#include <iostream>
#include <qpOASES.hpp>

namespace ocs2 {
namespace legged_robot {

using namespace std;
class SenvenOrderSpline : public Spline {
 public:
  using Node = QuinticSpline::Node;

  SenvenOrderSpline(const vector_t& coffe, scalar_t startTime, scalar_t endTime) {
    if(coffe.rows() != 8)
      throw std::runtime_error(" coffe of SenventhOrderSpline wrong! ");
    c_ = coffe;
    t0_ = startTime;
    t1_ = endTime;
    dt_ = t1_ - t0_;
  }

  scalar_t position(scalar_t time) const;

  scalar_t velocity(scalar_t time) const;

  scalar_t acceleration(scalar_t time) const;

  // scalar_t startTimeDerivative(scalar_t t) const;

  // scalar_t finalTimeDerivative(scalar_t t) const;

 private:
  scalar_t normalizedTime(scalar_t t) const;

  Eigen::Matrix<scalar_t, 8, 1> c_;
  scalar_t t0_, t1_;
  scalar_t dt_;

  // scalar_t dc0_;  // derivative w.r.t. dt_
  // scalar_t dc1_;  // derivative w.r.t. dt_
  // scalar_t dc2_;  // derivative w.r.t. dt_
  // scalar_t dc3_;  // derivative w.r.t. dt_
};

class SeventhOrderSplineCpg : public Spline {
public:
  SeventhOrderSplineCpg(SenvenOrderSpline& leftSpline, SenvenOrderSpline& rightSpline, scalar_t midTime)
   : leftSpline_(leftSpline), rightSpline_(rightSpline), midTime_(midTime) {}

  scalar_t position(scalar_t time) const override {
    return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
  }

  scalar_t velocity(scalar_t time) const override {
    return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
  }

  scalar_t acceleration(scalar_t time) const override {
    return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
  }
private:
  scalar_t midTime_;
  SenvenOrderSpline leftSpline_;
  SenvenOrderSpline rightSpline_;
};


class MinimumJerkSeventhOrderSplineSolver {
 public:
  using Node = QuinticSpline::Node;

  MinimumJerkSeventhOrderSplineSolver(scalar_t miniumOder);

  vector_t solveCoffectient(Node liftOff, scalar_t leftMidHeight, Node apex, scalar_t rightMidHeight, Node touchDown);
 private:

  int factorial(int x) {
      int fac = 1;
      for(int i = x; i > 0; i--)
          fac = fac * i;
      return fac;
  };
  const scalar_t miniumOrder_;
  qpOASES::QProblem qp_;
  Eigen::Matrix<scalar_t, 16, 16, Eigen::RowMajor> H_;
  Eigen::Matrix<scalar_t, 13, 16, Eigen::RowMajor> Aeq_;

  // scalar_t midTime_;
  // SevenOrderSpline leftSpline_;
  // SevenOrderSpline rightSpline_;
};

}  // namespace legged_robot
}  // namespace ocs2
