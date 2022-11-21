#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ModelSettings.h"

namespace ocs2 {
namespace legged_robot {

class LeggedRobotWithArmDynamicsAD final : public SystemDynamicsBase {
    public:
        LeggedRobotWithArmDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info, const std::string& modelName,
                                     const ModelSettings& modelSettings);
        
        ~LeggedRobotWithArmDynamicsAD() override = default;
        LeggedRobotWithArmDynamicsAD* clone() const override {return new LeggedRobotWithArmDynamicsAD(*this); }

        vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
        VectorFunctionLinearApproximation linearApproximation(scalar_t time, const vector_t& state, const vector_t& input, 
                                                              const PreComputation& preComp) override;

    private:
        LeggedRobotWithArmDynamicsAD(const LeggedRobotWithArmDynamicsAD& rhs) = default;

        PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;

};

} // namespace legged_robot
} // namespace ocs2