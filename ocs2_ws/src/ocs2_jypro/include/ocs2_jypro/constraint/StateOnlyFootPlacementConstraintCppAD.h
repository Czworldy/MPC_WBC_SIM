#pragma once

#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_jypro/constraint/EndEffectorLinearConstraint.h"
#include <ocs2_core/constraint/StateConstraintCppAd.h>

#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "ocs2_jypro/foot_planner/CubicSpline.h"



namespace ocs2 {
namespace legged_robot {

class StateOnlyFootPlacementConstraint  : public StateConstraintCppAd {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Config {
        explicit Config(
                        scalar_t hessianDiagonalShiftParam = 1e-6)
            :hessianDiagonalShift(hessianDiagonalShiftParam) {
            assert(hessianDiagonalShift >= 0.0);
        }
        scalar_t hessianDiagonalShift;
    };
    StateOnlyFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager, 
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                    const std::string& modelName, const std::string& modelFolder, bool recompile,
                                    Config config, size_t contactPointIndex,
                                    const size_t& stateDim);


    ~StateOnlyFootPlacementConstraint() override = default;
    StateOnlyFootPlacementConstraint* clone() const override { return new StateOnlyFootPlacementConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t time) const override;
    ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state,
                                       const ad_vector_t& parameters) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                        const PreComputation& preComp) const override;
    VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComp) const override;


    private:
        StateOnlyFootPlacementConstraint(const StateOnlyFootPlacementConstraint& other) = default;

        ad_vector_t getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, 
                               const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const ad_vector_t& state);

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        // std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        const PinocchioEndEffectorKinematicsCppAd& endEffectorKinematics_;


        const Config config_;
        const size_t contactPointIndex_;
        const size_t stateDim_;



        Eigen::Matrix<scalar_t, 6, 4> B;

        Eigen::Matrix<scalar_t, 6, 3> Ax;
        scalar_t tor = 0.05, stance_tol = 0.05;


        std::function<void(const ad_vector_t&, ad_vector_t&)> positionFunc_;
        const CubicSpline transitionSpline_;


};

} // namespace legged_robot
} // namespace ocs2