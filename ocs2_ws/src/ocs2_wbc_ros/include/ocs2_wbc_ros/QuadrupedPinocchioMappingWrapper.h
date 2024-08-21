/*
 * @Author: Jiyu Yu 
 * @Date: 2024-07-02 09:19:26 
 * @Last Modified by: Jiyu Yu
 * @Last Modified time: 2024-07-02 09:50:57
 */

#pragma once 

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_anymal_models/QuadrupedPinocchioMapping.h>

namespace ocs2 {

class QuadrupedPinocchioMappingWrapper : public PinocchioStateInputMapping<scalar_t> {
 public:
  QuadrupedPinocchioMappingWrapper(const anymal::QuadrupedPinocchioMapping& mapping) :
    mapping_(mapping) {};
  ~QuadrupedPinocchioMappingWrapper() override = default;
  QuadrupedPinocchioMappingWrapper* clone() const override {
    return new QuadrupedPinocchioMappingWrapper(mapping_);
  }

  vector_t getPinocchioJointPosition(const vector_t& state) const override {
    switched_model::joint_coordinate_t joint_coordinate = state.tail(switched_model::JOINT_COORDINATE_SIZE);
    return mapping_.getPinocchioJointVector(joint_coordinate);
  }
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override {
    switched_model::joint_coordinate_t joint_velocity = input.tail(switched_model::JOINT_COORDINATE_SIZE);
    return mapping_.getPinocchioJointVector(joint_velocity);
  }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, 
                                                const matrix_t& Jq, const matrix_t& Jv) const override {
    throw std::runtime_error("[QuadrupedPinocchioMappingWrapper] Not implemented yet.");                                              
  }
  
 private:
  const anymal::QuadrupedPinocchioMapping& mapping_;
};

} // namespace ocs2
