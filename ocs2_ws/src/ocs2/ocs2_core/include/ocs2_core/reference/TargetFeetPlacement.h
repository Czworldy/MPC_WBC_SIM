#pragma once

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>


namespace ocs2 {

//X20 (x,y) = (0.3377,0.2033)
#define __FOOT_X__ 0.3377
#define __FOOT_Y__ 0.2033
#define __FOOT_R__ 0.036
struct TargetFeetPlacement {
 using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  TargetFeetPlacement() : TargetFeetPlacement(
    std::vector<vector3_t>{{__FOOT_X__, __FOOT_Y__, __FOOT_R__},{-__FOOT_X__, __FOOT_Y__, __FOOT_R__}}, 
    std::vector<vector3_t>{{__FOOT_X__, -__FOOT_Y__, __FOOT_R__},{-__FOOT_X__, -__FOOT_Y__, __FOOT_R__}}) {}
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