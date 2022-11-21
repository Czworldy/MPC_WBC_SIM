#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/initialization/Initializer.h>

#include "SwitchedModelReferenceManager.h" 

namespace ocs2 {
namespace legged_robot {

class LeggedRobotWithArmInitializer final : public Initializer {
    public:
        /*
        * Constructor
        * @param [in] info : The centroidal model information.
        * @param [in] referenceManager : Switched system reference manager.
        * @param [in] extendNormalizedMomentum: If true, it extrapolates the normalized momenta; otherwise sets them to zero.
        */       
        LeggedRobotWithArmInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager, 
                                    bool extendNorimalizedMomentum = false);
    
        ~LeggedRobotWithArmInitializer() override = default;
        LeggedRobotWithArmInitializer* clone() const override;

        void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;
    
    private:
        LeggedRobotWithArmInitializer(const LeggedRobotWithArmInitializer& other) = default;

        const CentroidalModelInfo info_;
        const SwitchedModelReferenceManager* referenceManagerPtr_;
        const bool extendNormalizedMomentum_;
};

} // namespace legged_robot
} // namespace ocs2
