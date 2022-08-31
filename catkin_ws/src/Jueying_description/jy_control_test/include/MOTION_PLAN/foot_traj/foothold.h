#ifndef FOOTHOLD_H
#define FOOTHOLD_H

#include "Gait.h"
#include "cppTypes.h"
#include  "UserParameter.h"
#include "FootSwingTrajectory.h"
#include "ros/ros.h"
#include <fstream>

class FootHold{
    public:
        FootHold(Gait* gait, float dtSEG,UserParameter<float> & param);
        ~FootHold(){}

        void FootHoldPlan(Vec2<float> des_vel, 
                                                Vec2<float>* hip_vel,
                                                Vec31<float>* hip_pos,
                                                Vec31<float>* foot_p,
                                                            
                                                Vec2<float>* foothold_des,
                                                Vec31<float>* swingTraj_p,
                                                Vec31<float>* swingTraj_v,
                                                Vec31<float>* swingTraj_a);
        void RecordData();
    
    protected:
        Gait* _gait;
        Vec31<float> _desFootHold[4];
        float _dt;
        Vec2<float> _Kfd;
        float _dt_control_loop;
        float _p_rel_max;
        bool firstSwing[4];
        float swingTimeRemaining[4];
        float stanceTimeRemaining[4];

        FootSwingTrajectory<float> footSwingTrajectory[4];
        Vec41<float> contactStates, swingStates;
};


#endif