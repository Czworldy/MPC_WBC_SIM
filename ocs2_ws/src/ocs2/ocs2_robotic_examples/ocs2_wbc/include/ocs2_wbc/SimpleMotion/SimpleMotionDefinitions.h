#pragma once

#include "ocs2_jypro/common/Types.h"


namespace ocs2{
namespace wbc{
using namespace legged_robot;

struct OneLimbData {
	scalar_t value[3];
    void clear() {
        value[0] = 0.0;
        value[1] = 0.0;
        value[2] = 0.0;
    }
};

struct LimbsContacts {
public: 
    float lf;
    float rf;
    float lh;
    float rh;
};

struct LimbsPosVel {
	OneLimbData lf_pos;
	OneLimbData rf_pos;
	OneLimbData lh_pos;
	OneLimbData rh_pos;
	OneLimbData lf_vel;
	OneLimbData rf_vel;
	OneLimbData lh_vel;
	OneLimbData rh_vel;
  void clear(){
        lf_pos.clear();
        rf_pos.clear();
        lh_pos.clear();
        rh_pos.clear();
        lf_vel.clear();
        rf_vel.clear();
        lh_vel.clear();
        rh_vel.clear();
  }
  //[LF, LH, RF, RH]
  void set(const vector_t& q_j, const vector_t& dq_j){ 
    Eigen::Map<vector3_t>(lf_pos.value) = q_j.head(3);
    Eigen::Map<vector3_t>(lh_pos.value) = q_j.segment(3,3);
    Eigen::Map<vector3_t>(rf_pos.value) = q_j.segment(6,3);
    Eigen::Map<vector3_t>(rh_pos.value) = q_j.tail(3);

    Eigen::Map<vector3_t>(lf_vel.value) = dq_j.head(3);
    Eigen::Map<vector3_t>(lf_vel.value) = dq_j.segment(3,3);
    Eigen::Map<vector3_t>(rf_vel.value) = dq_j.segment(6,3);
    Eigen::Map<vector3_t>(rh_vel.value) = dq_j.head(3);
  }
  // [LF, RF, LH, RH]
  vector_t get_q_j() const{
    vector_t q_j(12);
    Eigen::Map<vector3_t>(q_j.data()) = Eigen::Map<const vector3_t>(lf_pos.value);
    Eigen::Map<vector3_t>(q_j.data()+3) = Eigen::Map<const vector3_t>(rf_pos.value);
    Eigen::Map<vector3_t>(q_j.data()+6) = Eigen::Map<const vector3_t>(lh_pos.value);
    Eigen::Map<vector3_t>(q_j.data()+9) = Eigen::Map<const vector3_t>(rh_pos.value);
    return q_j;
  }
  // [LF, RF, LH, RH]
  vector_t get_dq_j() const{
    vector_t dq_j(12);
    Eigen::Map<vector3_t>(dq_j.data()) = Eigen::Map<const vector3_t>(lf_vel.value);
    Eigen::Map<vector3_t>(dq_j.data()+3) = Eigen::Map<const vector3_t>(rf_vel.value);
    Eigen::Map<vector3_t>(dq_j.data()+6) = Eigen::Map<const vector3_t>(lh_vel.value);
    Eigen::Map<vector3_t>(dq_j.data()+9) = Eigen::Map<const vector3_t>(rh_vel.value);
    return dq_j;
  }
};

struct LimbsCommand
{
    OneLimbData lf_tau;
    OneLimbData rf_tau;
    OneLimbData lh_tau;
    OneLimbData rh_tau;
    
    OneLimbData lf_pos;
    OneLimbData rf_pos;
    OneLimbData lh_pos;
    OneLimbData rh_pos;
      
    OneLimbData lf_vel;
    OneLimbData rf_vel;
    OneLimbData lh_vel;
    OneLimbData rh_vel;
};

}
}