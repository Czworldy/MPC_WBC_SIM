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
};

struct LimbsCommand
{
    OneLimbData lf_tau;
    OneLimbData rf_tau;
    OneLimbData lh_tau;
    OneLimbData rh_tau;
};

}
}