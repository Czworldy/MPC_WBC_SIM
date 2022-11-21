#pragma once

#include "ros/ros.h"
#include "time.h"
#include <ros/node_handle.h>
#include <ros/package.h>

#include <utility>

#include <ocs2_core/cost/StateInputCostCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include "ModelSettings.h"

namespace ocs2 {

class LeggedRobotWithArmQuadraticEEPositionCost final : public StateInputCostCppAd {
    public: 
        LeggedRobotWithArmQuadraticEEPositionCost(matrix_t Q,
                                                  std::string endEffectorId,
                                                  size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                                  const std::string& modelFolder, bool recompileLibraries,
                                                  bool verbose);
        ~LeggedRobotWithArmQuadraticEEPositionCost() override = default;
        LeggedRobotWithArmQuadraticEEPositionCost* clone() const override {return new LeggedRobotWithArmQuadraticEEPositionCost(*this);}

        vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories) const override;
    
    protected:
        ad_scalar_t costFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, 
                                 const ad_vector_t& parameters) const override;
        LeggedRobotWithArmQuadraticEEPositionCost(const LeggedRobotWithArmQuadraticEEPositionCost& other);
    
    private:
        const std::string endEffectorId_;
        size_t endEffectorFrameId_;
        matrix_t Q_;
        legged_robot::ModelSettings modelSettings_;
        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
};  

} // namespace ocs2

