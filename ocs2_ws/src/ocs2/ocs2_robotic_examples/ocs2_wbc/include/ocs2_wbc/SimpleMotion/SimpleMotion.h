///czworldu 2023/04/21 16:00:00

#pragma once

#include "ocs2_wbc/SimpleMotion/SimpleMotionDefinitions.h"
#include "ocs2_wbc/UserParameter.h"
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include "ocs2_wbc/UserParameter.h"
#include "ocs2_wbc/SimpleMotion/TerrainEstimator.h"
// C++
#include <iostream>
#include <fstream>
#include <memory>

using namespace std;

namespace ocs2 {
namespace wbc {


class SimpleMotion {

public:
    SimpleMotion(const UserParameter& userParameter, bool verbose);
    ~SimpleMotion(){};

    void PDMotionRun(LimbsCommand& command);
    void PDSafeGuardRun(LimbsCommand& command);

    // PD 

    //[LF, LH, RF, RH]
    void update(const vector_t& q_j, const vector_t& dq_j) { currentLimbsStates_.set(q_j, dq_j); };
    void update(const LimbsPosVel& input) { currentLimbsStates_ = input; }
    bool PDSetUpMotion(scalar_t angle_haa, scalar_t angle_hfe, scalar_t angle_kfe, scalar_t timeGoal);
    void KeepLimbsAngles();
    bool isPDMotionFinished(){ 
        if (timeNowPD_ <= timeGoalPD_)  
            return false; 
        else 
            return true; 
    }


    // SafeGuard
    void PDSafeGuardSetUpMotion();

    //contact_flag lf lh rf rh 
    //eePos:{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}; 
    TerrainEstData TerrainEst(const Eigen::Vector4i& contact_flag, const std::vector<vector3_t>& eePos,
                              const Eigen::Quaternion<scalar_t>& baseOri);

private:
    // PD
    void PDSetUpInitialLimbsAngles();
    void PDSetUpGoalLimbsAngles(scalar_t angle_haa, scalar_t angle_hfe, scalar_t angle_kfe);
    void PDSetUpGoalTime(scalar_t timeGoal) { timeGoalPD_ = timeGoal; };
    void CubicSplinePlanForLimbs();
    vector3_t TrajectoryPlan_d(scalar_t startPoint, scalar_t finalPoint, scalar_t finalTime, scalar_t time_traj);


    const UserParameter& paramf;

    // PD
    long long int iteratorPD_;
    LimbsPosVel initialLimbsAngles_, goalLimbsAngles_, planedLimbsStates_;
    scalar_t timeGoalPD_, timeNowPD_, timeCycle_;
    bool isPDSetUp;

    // Input
    LimbsPosVel currentLimbsStates_;

    //print
    bool verbose_;
    //terrEst para
    TerrainEstimator terrEst;


};

}
}