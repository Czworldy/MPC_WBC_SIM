#pragma once

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>


namespace ocs2 {


struct TargetFeetPlacement {
 using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  TargetFeetPlacement() : TargetFeetPlacement(
    std::vector<vector3_t>{{-0.177,0.338,0.03},{-0.177,-0.338,0.03}}, 
    std::vector<vector3_t>{{0.177,0.338,0.03},{0.177,-0.338,0.03}}) {}
  TargetFeetPlacement(std::vector<vector3_t> Left, std::vector<vector3_t> Right)
    : targetFeetPlacemetLeft_(std::move(Left)), targetFeetPlacemetRight_(std::move(Right)) {}

  std::vector<vector3_t> targetFeetPlacemetLeft_;
  std::vector<vector3_t> targetFeetPlacemetRight_;
};         
// void swap(TargetFeetPlacement& lh, TargetFeetPlacement& rh){
//   lh.targetFeetPlacemetLeft_.swap(rh.targetFeetPlacemetLeft_);
//   rh.targetFeetPlacemetRight_.swap(rh.targetFeetPlacemetRight_);
// }
// std::ostream& operator<<(std::ostream& out, const TargetFeetPlacement& targetFeetPlacement) {
//   for(const auto& left : targetFeetPlacement.targetFeetPlacemetLeft_){
//     out << "left: " << left << "\n";
//   }
//   for(const auto& right : targetFeetPlacement.targetFeetPlacemetRight_){
//     out << "right: " << right << "\n";
//   }

//   return out;
// }

} // namespace ocs2