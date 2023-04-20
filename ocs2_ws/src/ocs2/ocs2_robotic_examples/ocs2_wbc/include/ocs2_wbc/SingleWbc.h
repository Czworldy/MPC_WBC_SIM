//
// Created by czworldy on 2023/4/17.
//

#pragma once

#include "ocs2_wbc/WbcBase.h"

namespace ocs2 {
namespace wbc{

class SingleWbc : public WbcBase {
public:
    SingleWbc(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                    const PinocchioEndEffectorKinematics &eeKinematics,
                    const std::string& paramFile);

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                    size_t mode,
                    scalar_t period, scalar_t time) override;
};
}
}
