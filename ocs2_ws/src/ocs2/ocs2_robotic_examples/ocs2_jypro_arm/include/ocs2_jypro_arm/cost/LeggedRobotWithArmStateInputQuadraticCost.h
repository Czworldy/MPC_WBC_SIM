#pragma once

#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

namespace ocs2 {
namespace legged_robot {

class LeggedRobotWithArmStateInputQuadraticCost final : public QuadraticStateInputCost {
    public:
        LeggedRobotWithArmStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info, 
                                                  const SwitchedModelReferenceManager& referenceManager);

        ~LeggedRobotWithArmStateInputQuadraticCost() override = default;
        LeggedRobotWithArmStateInputQuadraticCost* clone() const override;

    private:
        LeggedRobotWithArmStateInputQuadraticCost(const LeggedRobotWithArmStateInputQuadraticCost& rhs) = default;

        std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input, 
                                                             const TargetTrajectories& targetTrajectories) const override;

        const CentroidalModelInfo info_;
        const SwitchedModelReferenceManager* referenceManagerPtr_;                                                     

};

} // namespace legged_robot
} // namespace ocs2
