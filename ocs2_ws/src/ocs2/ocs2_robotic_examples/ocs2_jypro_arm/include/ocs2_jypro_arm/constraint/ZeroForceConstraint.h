#pragma once

#include "SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {
namespace legged_robot {

class ZeroForceConstraint final : public StateInputConstraint {
    public:
        /*
         * Constructor
         * @param [in] referenceManager : Switched model ReferenceManager.
         * @param [in] contactPointIndex : The 3 DoF contact index.
         * @param [in] info : The centroidal model information.
         */
        ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex, CentroidalModelInfo info);
        
        ~ZeroForceConstraint() override = default;
        ZeroForceConstraint* clone() const override { return new ZeroForceConstraint(*this); }

        bool isActive(scalar_t time) const override;
        size_t getNumConstraints(scalar_t time) const override {return 3;}
        vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input, 
                                                                    const PreComputation& preComp) const override;

    private:
        ZeroForceConstraint(const ZeroForceConstraint& other) = default;

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        const size_t contactPointIndex_;
        const CentroidalModelInfo info_;
};

} // namespace legged_robot
} // namespace ocs2