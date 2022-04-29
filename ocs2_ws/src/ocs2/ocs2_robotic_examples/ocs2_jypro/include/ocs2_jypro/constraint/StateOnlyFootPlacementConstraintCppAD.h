#pragma once

#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_jypro/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace legged_robot {

class StateOnlyFootPlacementConstraint final : public StateInputConstraint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Config {
        explicit Config(scalar_t gamma = 0.7,
                        scalar_t hessianDiagonalShiftParam = 1e-6)
            : gamma(gamma),
            hessianDiagonalShift(hessianDiagonalShiftParam) {
            assert(gamma > 0.0);
            assert(hessianDiagonalShift >= 0.0);
        }

        scalar_t gamma;
        scalar_t hessianDiagonalShift;
    };
    StateOnlyFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager, 
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                    Config config, size_t contactPointIndex,
                                    CentroidalModelInfo info);

    ~StateOnlyFootPlacementConstraint() override = default;
    StateOnlyFootPlacementConstraint* clone() const override { return new StateOnlyFootPlacementConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t time) const override { return 6; };
    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const PreComputation& preComp) const override;
    VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

    
    private:
        StateOnlyFootPlacementConstraint(const StateOnlyFootPlacementConstraint& other) = default;

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;

        const Config config_;
        const size_t contactPointIndex_;
        const CentroidalModelInfo info_;

        Eigen::Matrix<size_t, 6, 4> B;

};

} // namespace legged_robot
} // namespace ocs2