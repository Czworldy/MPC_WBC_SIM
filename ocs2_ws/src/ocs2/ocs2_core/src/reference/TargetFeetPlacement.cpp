#include "ocs2_core/reference/TargetFeetPlacement.h"

#include <ocs2_core/misc/Display.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void TargetFeetPlacement::init(){
    targetFeetPlacemetLeftFront_ = std::vector<vector3_t>{{__FOOT_X__, __FOOT_Y__, __FOOT_R__}};
    targetFeetPlacemetRightFront_ = std::vector<vector3_t>{{__FOOT_X__, -__FOOT_Y__, __FOOT_R__}}; 
    targetFeetPlacemetLeftBack_ = std::vector<vector3_t>{{-__FOOT_X__, __FOOT_Y__, __FOOT_R__}};
    targetFeetPlacemetRightBack_ = std::vector<vector3_t>{{-__FOOT_X__, -__FOOT_Y__, __FOOT_R__}};
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void TargetFeetPlacement::clear() {
  targetFeetPlacemetLeftFront_.clear();
  targetFeetPlacemetRightFront_.clear();
  targetFeetPlacemetLeftBack_.clear();
  targetFeetPlacemetRightBack_.clear();
}

}  // namespace ocs2
