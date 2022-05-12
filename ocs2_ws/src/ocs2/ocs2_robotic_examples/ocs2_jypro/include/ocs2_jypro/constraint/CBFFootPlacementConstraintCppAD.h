#pragma once

#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_jypro/constraint/EndEffectorLinearConstraint.h"
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include "ocs2_jypro/dynamics/LeggedRobotDynamicsAD.h"



namespace ocs2 {
namespace legged_robot {

class CBFFootPlacementConstraint  : public StateInputConstraintCppAd {
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
    CBFFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager, 
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                    const CentroidalModelInfo& info,
                                    const std::string& modelName,
                                    Config config, size_t contactPointIndex
                                    );


    ~CBFFootPlacementConstraint() override = default;
    CBFFootPlacementConstraint* clone() const override { return new CBFFootPlacementConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t time) const override { return 6; };
    ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                       const ad_vector_t& parameters) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const PreComputation& preComp) const override;
    VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;


    private:
        CBFFootPlacementConstraint(const CBFFootPlacementConstraint& other);

        ad_vector_t getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, 
                               const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const ad_vector_t& state);
        std::vector<ad_matrix_t> getJacobiCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                          const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                          const ad_vector_t& state);
        ad_vector_t getValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                        const CentroidalModelPinocchioMappingCppAd& mappingCppAd, const ad_vector_t& state,
                        const ad_vector_t& input);

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        const PinocchioEndEffectorKinematicsCppAd& endEffectorKinematics_;
        const CentroidalModelInfo& info_;

        const Config config_;
        const size_t contactPointIndex_;

        Eigen::Matrix<scalar_t, 6, 4> B;

        Eigen::Matrix<scalar_t, 6, 3> Ax;

        std::function<void(const ad_vector_t&, ad_vector_t&)> positionFunc_;
        std::function<void(const ad_vector_t&, ad_matrix_t&)> JacobiFunc_;
        std::function<void(const ad_vector_t&, ad_vector_t&)> systemFlowMapFunc_;

};

} // namespace legged_robot
} // namespace ocs2