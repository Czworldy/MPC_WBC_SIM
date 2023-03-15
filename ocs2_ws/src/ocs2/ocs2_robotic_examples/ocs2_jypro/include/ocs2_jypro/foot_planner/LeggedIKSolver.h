// This class is for calculate inverse kinematics
// we init it once to save a joint angle value to warm start the next calculation
#pragma once
#include "ocs2_jypro/common/Types.h"
#include <ocs2_jypro/gait/MotionPhaseDefinition.h>


namespace ocs2 {
namespace legged_robot {

class LeggedIKSolver {
    using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
    using matrix4_t = Eigen::Matrix<scalar_t, 4, 4>;
    public:
        LeggedIKSolver(const vector3_t& linkLengths, scalar_t xBodyLength, scalar_t yBodyLength);

        // solve ik for individual leg
        vector3_t solveIK(const vector3_t& footPositionInWorldFrame, int leg_id);
        void setBodyState(const vector6_t& bodyPose);
        const matrix4_t& getBodyTfMatrix() const {return _O_B_tfMatrix_;}
        
    private:
        vector3_t inverseKinematics(const vector3_t& footPositionInShoulderFrame);
        const vector3_t linkLengths_;
        // half body length
        const scalar_t xBodyLength_, yBodyLength_, haa_hfe_y = 0.12325;
        matrix4_t _O_B_tfMatrix_;
        // PinocchioInterface& pino_interface_;

};

}// namespace legged_robot
}// namespace ocs2