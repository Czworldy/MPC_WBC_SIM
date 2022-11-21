#pragma once


#include <pinocchio/fwd.hpp>
#include <PinocchioInterfaceForArm.h>
#include <ArmSettings.h>
#include <prePlanDefinition.h>

namespace ocs2 {
namespace legged_robot {
namespace arm {

class TaskPriorityControl final {

    public:
        TaskPriorityControl();
        ~TaskPriorityControl(){};

        void TPcontrolLaw(const vector_t& jointSpacePosition, 
                          const GripperBaseVelocity& endEffectorVelocity, 
                          vector_t& jointSpaceVelocity);
    
    private:
        void taskPrioritySetUp(const GripperBaseVelocity& endEffectorVelocity);
        void nullSpaceProjectorIteration(const matrix_t& jacobi, matrix_t& nullSpaceMat);
        matrix_t pseudoInverse(matrix_t const& matrix, scalar_t sigmaThreshold = 0.001);
        matrix_t nullSpaceCal(const matrix_t& A);

        std::unique_ptr<PinocchioInterfaceForArm> pinocchioInterfacePtr_;
        ArmSettings armSettings_;
        matrix_t gripperJacobi_, baseJacobi_;
        matrix_t jacobiTaskA_, jacobiTaskB_, jacobiTaskC_, jacobiTaskD_;
        vector_t velTaskA_, velTaskB_, velTaskC_, velTaskD_;
        matrix_array_t jacobiVector_;
        vector_array_t velocityVector_;
        matrix_t nullSpaceProjector_;
        int taskNum_;
};

} // namespace arm
}
} // namespace ocs2