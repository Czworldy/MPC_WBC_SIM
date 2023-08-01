/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_jypro/LeggedRobotPinocchioMapping.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR> 
LeggedRobotPinocchioMappingTpl<SCALAR>::LeggedRobotPinocchioMappingTpl(CentroidalModelInfoTpl<SCALAR> info)
    : pinocchioInterfacePtr_(nullptr), centroidalModelInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
LeggedRobotPinocchioMappingTpl<SCALAR>* LeggedRobotPinocchioMappingTpl<SCALAR>::clone() const {
    return new LeggedRobotPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto LeggedRobotPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
    return state.tail(centroidalModelInfo_.stateDim - 6);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto LeggedRobotPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
    // throw std::runtime_error("LeggedRobotPinocchioMappingTpl::getPinocchioJointVelocity not implemented yet!");
    // return input.tail(modelInfo_.inputDim - 6);
    const auto& model = pinocchioInterfacePtr_->getModel();
    const auto& data = pinocchioInterfacePtr_->getData();
    const auto& info = centroidalModelInfo_;
    assert(info.stateDim == state.rows());
    assert(info.inputDim == input.rows());

    const auto& A = getCentroidalMomentumMatrix(*pinocchioInterfacePtr_);
    const Eigen::Matrix<SCALAR, 6, 6> Ab = A.template leftCols<6>();
    const auto Ab_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);

    const auto jointVelocities = centroidal_model::getJointVelocities(input, info).head(info.actuatedDofNum);

    Eigen::Matrix<SCALAR, 6, 1> momentum = info.robotMass * centroidal_model::getNormalizedMomentum(state, info);
    if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
        momentum.noalias() -= A.rightCols(info.actuatedDofNum) * jointVelocities;
    }

    vector_t vPinocchio(info.generalizedCoordinatesNum);
    vPinocchio.template head<6>().noalias() = Ab_inv * momentum;
    vPinocchio.tail(info.actuatedDofNum) = jointVelocities;

    return vPinocchio;
}

/******************************************************************************************************/
/********************************************************************s**********************************/
/******************************************************************************************************/
template <typename SCALAR>
auto LeggedRobotPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {

    matrix_t dfdx = matrix_t::Zero(Jq.rows(), centroidalModelInfo_.stateDim);
    matrix_t dfdu = matrix_t::Zero(Jv.rows(), centroidalModelInfo_.inputDim);

    dfdx.middleCols(6, centroidalModelInfo_.generalizedCoordinatesNum) = Jq;
    return {dfdx, dfdu};

}

// explicit template instantiation
template class ocs2::legged_robot::LeggedRobotPinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::legged_robot::LeggedRobotPinocchioMappingTpl<ocs2::ad_scalar_t>;

} // namespace legged_robot
} // namespace ocs2