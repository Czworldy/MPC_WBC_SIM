#ifndef SIMPLEMOTION_H
#define SIMPLEMOTION_H

// WBC
#include "quadruped_dynamics_model.h"
#include "wbc_ctrl.h"
#include "UserParameter.h"
#include "cppTypes.h"
#include "wbc_definitions.h"
#include "TerrainEstimator.h"
// C++
#include <iostream>
#include <fstream>
#include <memory>

using namespace std;

class SimpleMotion {

public:
    SimpleMotion(bool verbose);
    ~SimpleMotion(){
        in_x.close();
        in_y.close();
        in_z.close();
        in_roll.close();
        in_pitch.close();
        in_yaw.close();

        in_x_vel.close();
        in_y_vel.close();
        in_z_vel.close();
        in_roll_vel.close();
        in_pitch_vel.close();
        in_yaw_vel.close();

        in_foot_lf_x.close();
        in_foot_lf_y.close();
        in_foot_lf_z.close();

        in_foot_rf_x.close();
        in_foot_rf_y.close();
        in_foot_rf_z.close();

        in_foot_lh_x.close();
        in_foot_lh_y.close();
        in_foot_lh_z.close();

        in_foot_rh_x.close();
        in_foot_rh_y.close();
        in_foot_rh_z.close();
    };

    void PDMotionRun(LimbsCommand& command);
    void WBCMotionRun(LimbsCommand& command, bool& safeGuard);
    void MPCWBCRun(float time_stamp, LimbsCommand& command, bool& safeGuard);
    void PDSafeGuardRun(LimbsCommand& command);

    // PD 
    // void LimbsStatesInput(const LimbsPosVel& input);
    bool PDSetUpMotion(double angle_haa, double angle_hfe, double angle_kfe, double timeGoal);
    void KeepLimbsAngles();
    bool isPDMotionFinished();

    // WBC
    void EstimatedStatesInput(const EstimatorOutput& input);
    bool WBCSetUpBaseMotion(float x, float y, float z, float roll, float pitch, float yaw, float timeGoal);
    void KeepBaseStates();
    bool isWBCMotionFinished();
    bool WBCSetUpSwingFootMotion(float base_x, float base_y, float base_z, float base_roll, float base_pitch, float base_yaw,
                                 size_t foot_id, float foot_x, float foot_y, float foot_z,
                                 float timeGoal);
    void KeepSwingFootStates(); 
    void UpdateMPCMsg(conversionData* mpcMsg, float time_stamp);
    void UpdateControlFrame(const EstimatorOutput& input);
    const std::vector<Vec31<float>> RecordData();                 

    // SafeGuard
    void PDSafeGuardSetUpMotion();

    void TerrainEst(const Vec41<int>&);

private:
    // PD
    void PDSetUpInitialLimbsAngles();
    void PDSetUpGoalLimbsAngles(double angle_haa, double angle_hfe, double angle_kfe);
    void PDSetUpGoalTime(double timeGoal);
    void CubicSplinePlanForLimbs();
    Vec31<double> TrajectoryPlan_d(double startPoint, double finalPoint, double finalTime, double time_traj);

    // WBC
    void WBCSetUpInitialBaseStates();
    void WBCSetUpGoalBaseStates(float x, float y, float z, float roll, float pitch, float yaw);
    void WBCSetUpGoalTime(float timeGoal);
    void WBCSetUpContactForBaseMotion();
    void CubicSplinePlanForBase();

    void WBCSetUpContactForSwingMotion(size_t foot_id);
    void WBCSetUpSwingFootInitialStates();
    void WBCSetUpSwingFootGoalStatesAll(float base_x, float base_y, float base_z, float base_roll, float base_pitch, float base_yaw,
                                        float foot_x, float foot_y, float foot_z);
    void CubicSplinePlanForSwingFoot();
    Vec31<float> TrajectoryPlan_f(float startPoint, float finalPoint, float finalTime, float time_traj);

    double timeCycle_;
    UserParameter<float> paramf;

    // PD
    long long int iteratorPD_;
    LimbsPosVel initialLimbsAngles_, goalLimbsAngles_, planedLimbsStates_;
    double timeGoalPD_, timeNowPD_;
    bool isPDSetUp;

    // WBC
    long long int iteratorWBC_;
    BaseStatesForPlan initialBaseStates_, goalBaseStates_, planedBaseStates_;
    FootStatesForPlan initialFootStates_, goalFootStates_, planedFootStates_;
    double timeGoalWBC_, timeNowWBC_;
    bool isWBCSetUp;
    bool isWBCSwingSetUp;
    LocomotionCtrlData<float> desiredDataWBC_;
    QuadrupedDynamicsModel jueying_;
    std::shared_ptr<WBC_Ctrl<float>> wbc_ctrl_;
    DVec<float> tauWBC_;
    Eigen::Matrix<float,18,1> Q_;
    Eigen::Matrix<float,18,1> QDot_;
    size_t foot_id_;

    // Input
    LimbsPosVel currentLimbsStates_;
    ControlFSMData<float> currentStatesWBC_; 

    // mpc
    conversionData* mpcMsgPtr_ = nullptr;
    size_t indexMPCStateTime_;
    size_t indexMPCSwitchTime_;
    float timeUpdateMPC_;
    
    // Record Data
    ofstream in_x, in_y, in_z;
    ofstream in_roll, in_pitch, in_yaw;
    ofstream in_x_vel, in_y_vel, in_z_vel;
    ofstream in_roll_vel, in_pitch_vel, in_yaw_vel;
    ofstream in_foot_lf_x, in_foot_lf_y, in_foot_lf_z;
    ofstream in_foot_rf_x, in_foot_rf_y, in_foot_rf_z;
    ofstream in_foot_lh_x, in_foot_lh_y, in_foot_lh_z;
    ofstream in_foot_rh_x, in_foot_rh_y, in_foot_rh_z;

    ofstream in_tor_haa, in_tor_hfe, in_tor_kfe;

    std::vector<Vec31<float>> feet_result;

    //print
    bool verbose_;
    //terrEst para
    TerrainEstimator terrEst;

    double slope_delta_roll;
    double slope_delta_pitch;

};

#endif