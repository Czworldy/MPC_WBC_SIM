#pragma once

#include <utility>

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {

/** Quadratic end-effector position cost term */
class QuadraticEndEffectorPositionCostCppAd : public StateInputCost {
    public:
        QuadraticEndEffectorPositionCostCppAd(matrix_t Q,
                                              const PinocchioInterface& pinocchioInterface,
                                              const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                              std::string endEffectorId,
                                              size_t stateDim, size_t inputDim, const std::string& modelName,
                                              const std::string& modelFolder, bool recompileLibraries,
                                              bool verbose);
        ~QuadraticEndEffectorPositionCostCppAd() override = default;
        QuadraticEndEffectorPositionCostCppAd* clone() const override;

        /** Get cost term value */
        scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                          const PreComputation&) const final;
        
        /** Get cost term quadratic approximation */
        ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                       const TargetTrajectories& targetTrajectories,
                                                                       const PreComputation&) const final;

    protected:
        QuadraticEndEffectorPositionCostCppAd(const QuadraticEndEffectorPositionCostCppAd& rhs);

        /** Computes the end-effector position deviation around the norminal end-effector position.
         * This method can be overwritten if desiredTrajectory has a different dimensions. */
        ad_scalar_t quadraticEndEffectorPositionDeviationCppAd(const ad_vector_t& state, const ad_vector_t& desiredEEPosition,
                                                               PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                               PinocchioStateInputMapping<ad_scalar_t>& mapping);
        
        const std::string endEffectorId_;
        size_t endEffectorFrameId_;
        std::unique_ptr<ocs2::CppAdInterface> adInterfacePtr_;
        const ad_matrix_t Q_;
};

} // namespace ocs2