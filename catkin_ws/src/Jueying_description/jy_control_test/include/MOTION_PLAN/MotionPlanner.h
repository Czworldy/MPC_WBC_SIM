#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include "cppTypes.h"
#include "UserParameter.h"
#include "ControlFSMData.h"
#include "MotionPlanData.h"

#include "Gait.h"
#include "foothold.h"
#include "FootSwingTrajectory.h"
#include "SupportPolygon.h"

#include "AccelerationMin.h"
#include "DeviationFromPre.h"
#include "PathRegularization.h"
#include "SoftFinalConstraints.h"
#include "SearchOptCoefficients.h"

#include "quadruped_dynamics_model.h"

#include "osqp.h"
#include <fstream>

class MotionPlanner{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MotionPlanner(int nSegment, int iterationsBetweenSEG, double dt, 
                                         UserParameter<float> & paramf,
                                         UserParameter<double>& paramd);
        ~MotionPlanner();

        void initialize();
        
        void run(ControlFSMData<float>& data, DesMotionData& result);

        Vec2<float> getCoMDesPosition(double time);//函数未测试！！！
        Vec2<float> getCoMDesVelocity(double time);//函数未测试！！！
        Vec2<float> getCoMDesAcceleration(double time);//函数未测试！！！

    
    //private:
        void _SetupCommand(ControlFSMData<float>& data);
        void _RecordData();
        void _UpdateModel(const BodyStateEstData<float> & bodyEst,                               
                          const LegStateEstData<float> * legEst);

        DVec<double> CoMSpline_;
        DVec<double> CoMSpline_Pre_;

        //int nSegment_;
        int iterationsBetweenSEG_;
        int nSpline_;
        double dt_;
        double t_loop_;
        UserParameter<float> paramf_;
        UserParameter<double> paramd_;
        long long int iterationCounter_;
        long long int iter_gait_;
        long long int iter_gait_pre_;
        bool firstRun = true;
        bool firstSwing[4];

        float yaw_turn_rate_;
        float yaw_des_;

        float roll_des_;
        float pitch_des_;

        float x_vel_des_ = 0;
        float y_vel_des_ = 0;

        float body_height_ = 0.38;
        bool if_planned;

        Vec2<float> footHoldDes_[4];
        FootStateData footStateCur_;
        DMat<int> gaitTable_;

        c_float* P_Acc;
        c_float* P_Devia;
        c_float* P_Path;
        c_float* P_Final;
        c_float* P_;
        c_float* q_Devia;
        c_float* q_Path;
        c_float* q_Final;
        c_float* q_;

        Gait walking_;
        FootHold footHold_;
        Vec31<float> swingTraj_p_[4];
        Vec31<float> swingTraj_v_[4];
        Vec31<float> swingTraj_a_[4];
        SupportPolygon supportPolygon_;
        float line_margin;

        AccelerationMin accelerationMin_;
        DeviationFromPre deviationFromPre_;
        PathRegularization pathRegularization_;
        SoftFinalConstraints softFinalConstraints_;
        SearchOptCoefficients searchCoMCoeff_;

        QuadrupedDynamicsModel jueying;
        FBModelState<double> state_;

        DesMotionData result_;
};
#endif