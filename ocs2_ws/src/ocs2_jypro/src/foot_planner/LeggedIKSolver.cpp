

#include "ocs2_jypro/foot_planner/LeggedIKSolver.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

// Boost
#include <ocs2_jypro/common/ModelSettings.h>


namespace ocs2 {
namespace legged_robot {
    LeggedIKSolver::LeggedIKSolver(const vector3_t& linkLengths, scalar_t xBodyLength, scalar_t yBodyLength) 
    : linkLengths_(linkLengths),
    xBodyLength_(xBodyLength), yBodyLength_(yBodyLength) {
        _O_B_tfMatrix_.setIdentity();
    }

    void LeggedIKSolver::setBodyState(const vector6_t& bodyPose) {
        const vector3_t& bodyPosition = bodyPose.head(3);
        const vector3_t& bodyEulerAngles = bodyPose.tail(3);
        _O_B_tfMatrix_.topLeftCorner(3,3) = ocs2::getRotationMatrixFromZyxEulerAngles(bodyEulerAngles);
        _O_B_tfMatrix_.topRightCorner(3,1) = bodyPosition;
    }

    vector3_t LeggedIKSolver::solveIK(const vector3_t& footPositionInWorldFrame, int leg_id) {
        // transform foot position to shoulder frame
        const auto& footPositionInBodyFrame = _O_B_tfMatrix_.inverse() * footPositionInWorldFrame.homogeneous();
        // solve ik
        //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
        matrix4_t _B_S_tfMatrix;
        _B_S_tfMatrix.setIdentity();
        switch (leg_id) {
        case 0://LF_FOOT
            _B_S_tfMatrix(2,2) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << xBodyLength_, yBodyLength_, 0;
            break;
        case 1://RF_FOOT
            _B_S_tfMatrix(2,2) = -1; _B_S_tfMatrix(1,1) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << xBodyLength_, -yBodyLength_, 0;
            break;
        case 2://LH_FOOT
            _B_S_tfMatrix(2,2) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << -xBodyLength_, yBodyLength_, 0;
            break;
        case 3://RH_FOOT
            _B_S_tfMatrix(1,1) = -1; _B_S_tfMatrix(2,2) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << -xBodyLength_, -yBodyLength_, 0;
            break;
        default:
            throw std::runtime_error("LeggedIKSolver::solveIK: leg_id is not valid");
            break;
        }
        vector3_t footPositionInShoulderFrame = (_B_S_tfMatrix.inverse() * footPositionInBodyFrame).head(3);
        // std::cout <<"leg: " << leg_id << " in world: " << footPositionInWorldFrame.transpose() << " in shoulder: " << footPositionInShoulderFrame.transpose() << std::endl;   
        vector3_t angles = inverseKinematics(footPositionInShoulderFrame);
        // add offset
        return angles;
    }

    vector3_t LeggedIKSolver::solveIK(const vector6_t& bodyPose, const vector3_t& footPositionInWorldFrame, int leg_id) {
        const vector3_t& bodyPosition = bodyPose.head(3);
        const vector3_t& bodyEulerAngles = bodyPose.tail(3);
        matrix4_t _O_B_tfMatrixLocal = matrix4_t::Identity();
        _O_B_tfMatrixLocal.topLeftCorner(3,3) = ocs2::getRotationMatrixFromZyxEulerAngles(bodyEulerAngles);
        _O_B_tfMatrixLocal.topRightCorner(3,1) = bodyPosition;
        // transform foot position to shoulder frame
        const auto& footPositionInBodyFrame = _O_B_tfMatrixLocal.inverse() * footPositionInWorldFrame.homogeneous();
        // solve ik
        //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
        std::cout << "bodyPose: " << bodyPose.transpose() << std::endl;
        matrix4_t _B_S_tfMatrix;
        _B_S_tfMatrix.setIdentity();
        switch (leg_id) {
        case 0://LF_FOOT
            _B_S_tfMatrix(2,2) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << xBodyLength_, yBodyLength_, 0;
            break;
        case 1://RF_FOOT
            _B_S_tfMatrix(2,2) = -1; _B_S_tfMatrix(1,1) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << xBodyLength_, -yBodyLength_, 0;
            break;
        case 2://LH_FOOT
            _B_S_tfMatrix(2,2) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << -xBodyLength_, yBodyLength_, 0;
            break;
        case 3://RH_FOOT
            _B_S_tfMatrix(1,1) = -1; _B_S_tfMatrix(2,2) = -1;
            _B_S_tfMatrix.topRightCorner(3,1) << -xBodyLength_, -yBodyLength_, 0;
            break;
        default:
            throw std::runtime_error("LeggedIKSolver::solveIK: leg_id is not valid");
            break;
        }
        vector3_t footPositionInShoulderFrame = (_B_S_tfMatrix.inverse() * footPositionInBodyFrame).head(3);
        std::cout <<"leg: " << leg_id << " in world: " << footPositionInWorldFrame.transpose() << " in shoulder: " << footPositionInShoulderFrame.transpose() << std::endl;   
        vector3_t angles = inverseKinematics(footPositionInShoulderFrame);
        // add offset
        return angles;
    }

    vector3_t LeggedIKSolver::inverseKinematics(const vector3_t& footPositionInShoulderFrame) {
        vector3_t angles;
        scalar_t x, y, z;
        scalar_t l0, l1, l2;
        scalar_t s1, s2, s3;
        l0 = linkLengths_[0]; l1 = linkLengths_[1]; l2 = linkLengths_[2];
        x = footPositionInShoulderFrame[0]; y = footPositionInShoulderFrame[1]; z = footPositionInShoulderFrame[2];

        s1 = atan(y / z) - asin(l0 / sqrt(y*y + z*z));

        s3 = acos((x*x + y*y + z*z - l0*l0 - l1*l1 - l2*l2) / (2 * l1*l2));

        s2 = asin(x / sqrt((l1 + l2*cos(s3))*(l1 + l2*cos(s3)) + l2*sin(s3)*l2*sin(s3))) - atan(l2*sin(s3) / (l1 + l2*cos(s3)));
        angles << s1, s2, s3;

        return angles;
    }
    


}// namespace legged_robot
}// namespace ocs2
