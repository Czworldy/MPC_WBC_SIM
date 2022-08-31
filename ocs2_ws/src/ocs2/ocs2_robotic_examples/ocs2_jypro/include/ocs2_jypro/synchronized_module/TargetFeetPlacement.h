#pragma once

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>


namespace ocs2 {


struct TargetFeetPlacement {
 using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  TargetFeetPlacement() : TargetFeetPlacement(std::vector<vector3_t>{{0.,0.,0.}}, std::vector<vector3_t>{{0.,0.,0.}}) {}
  TargetFeetPlacement(std::vector<vector3_t> Left, std::vector<vector3_t> Right)
    : targetFeetPlacemetLeft_(std::move(Left)), targetFeetPlacemetRight_(std::move(Right)) {}

  std::vector<vector3_t> targetFeetPlacemetLeft_;
  std::vector<vector3_t> targetFeetPlacemetRight_;
};         
} // namespace ocs2