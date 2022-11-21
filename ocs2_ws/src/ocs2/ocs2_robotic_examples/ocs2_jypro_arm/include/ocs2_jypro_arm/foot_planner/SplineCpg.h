#pragma once

#include "CubicSpline.h"

namespace ocs2 {
namespace legged_robot {

class SplineCpg {
    public:
        SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown);

        scalar_t position(scalar_t time) const;

        scalar_t velocity(scalar_t time) const;

        scalar_t acceleration(scalar_t time) const;

        scalar_t startTimeDerivative(scalar_t time) const;

        scalar_t finalTimeDerivative(scalar_t time) const;

    private:
        scalar_t midTime_;
        CubicSpline leftSpline_;
        CubicSpline rightSpline_;
};

} // namespace legged_robot
} // namespace ocs2