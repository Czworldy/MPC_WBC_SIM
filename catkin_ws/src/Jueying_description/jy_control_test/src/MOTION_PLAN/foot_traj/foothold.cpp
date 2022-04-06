#include "foothold.h"

using namespace std;

FootHold::FootHold(Gait* gait, float dtSEG, UserParameter<float> & param){
    _gait = gait;
    _dt = dtSEG;
    _Kfd = param.Kfd;
    _dt_control_loop = param.t_plan_control_loop;
    _p_rel_max = param.p_rel_max;

    float timeStance(_gait->getCurrentStanceTime(_dt, 0));
    float timeSwing(_gait->getCurrentSwingTime(_dt, 0));

    for(int i(0); i<4; i++){
        swingTimeRemaining[i] = timeSwing;
        stanceTimeRemaining[i] = timeStance;
    }
}

void FootHold::FootHoldPlan(Vec2<float> des_vel, 
                            Vec2<float>* hip_vel,
                            Vec31<float>* hip_pos,
                            Vec31<float>* foot_p,
                        
                            Vec2<float>* foothold_des,
                            Vec31<float>* swingTraj_p,
                            Vec31<float>* swingTraj_v,
                            Vec31<float>* swingTraj_a){
                                                                                                          
    float timeStance(_gait->getCurrentStanceTime(_dt, 0));
    float timeSwing(_gait->getCurrentSwingTime(_dt, 0));
    DMat<int> pattern(_gait->getGaitPatternMat());

    contactStates = _gait->getContactState();
    swingStates = _gait->getSwingState();

    float pfx_rel, pfy_rel;
    
    for(int i(0); i<4; i++){
        if(firstSwing[i]){
            swingTimeRemaining[i] = timeSwing;
            stanceTimeRemaining[i] -=_dt_control_loop; 
        }
        else{
            swingTimeRemaining[i] -= _dt_control_loop;
            stanceTimeRemaining[i] = timeStance;
        }

        // cout << "\n" << endl;
        // cout << "foot id: " << i << endl;

        if(pattern(i,0)){//Stance
            //foothold_des[i][0] = foot_p[i][0] + des_vel[0]*(stanceTimeRemaining[i]+swingTimeRemaining[i]);
            foothold_des[i][0] = hip_pos[i][0] + des_vel[0]*(stanceTimeRemaining[i]+swingTimeRemaining[i]);
            foothold_des[i][1] = hip_pos[i][1] + des_vel[1]*(stanceTimeRemaining[i]+swingTimeRemaining[i]);

            // cout << "contact state: " << "stance" << endl;
        }
        else{//Swing
            //foothold_des[i][0] = foot_p[i][0] + des_vel[0]*(swingTimeRemaining[i]);
            foothold_des[i][0] = hip_pos[i][0] + des_vel[0]*(swingTimeRemaining[i]);
            foothold_des[i][1] = hip_pos[i][1] + des_vel[1]*(swingTimeRemaining[i]);

            // cout << "contact state: " << "swing" << endl;
        }
        
        //Print Data
        // cout << "timeSwing: " << timeSwing << endl;
        // cout << "timeStance: " << timeStance << endl;
        // cout << "swingTimeRemaining: " << swingTimeRemaining[i] << endl;
        // cout << "stanceTimeRemaining: " << stanceTimeRemaining[i] << endl;
        // cout << "hip_pos: " << endl;
        // cout << hip_pos[i][0] << " " << hip_pos[i][1] << hip_pos[i][2] << endl;
        // cout << "des_vel: " << endl;
        // cout << des_vel[0] << " " << des_vel[1] << endl;
        // cout << "foot_hold_des_A: " << endl;
        // cout <<  foothold_des[i][0] << " " << foothold_des[i][1] << endl;
        
        pfx_rel = 0.5*des_vel[0]*timeStance - _Kfd[0]*(des_vel[0] - hip_vel[i][0])*sqrt(hip_pos[i][2]/9.81f);
        pfy_rel = 0.5*des_vel[1]*timeStance - _Kfd[1]*(des_vel[1] - hip_vel[i][1])*sqrt(hip_pos[i][2]/9.81f);

        // cout << "hip_vel: " << endl;
        // cout << hip_vel[i][0] << " " << hip_vel[i][1] << endl;
        // cout << "foot_hold_des_B: " << endl;
        // cout <<  pfx_rel << " " << pfy_rel << endl;

        pfx_rel = fminf(fmax(pfx_rel, -_p_rel_max), _p_rel_max);
        pfy_rel = fminf(fmax(pfy_rel, -_p_rel_max), _p_rel_max);

        foothold_des[i][0] += pfx_rel;
        foothold_des[i][1] += pfy_rel;
    
        // cout << "foot_hold_des_all: " << endl;
        // cout <<  foothold_des[i][0] << " " << foothold_des[i][1] << endl;
        // cout << "\n" << endl;

        _desFootHold[i].head(2) = foothold_des[i];
        _desFootHold[i][2] = -0.003;//参数待调
        footSwingTrajectory[i].setFinalPosition(_desFootHold[i]);
        footSwingTrajectory[i].setHeight(0.06);
    }

    for(int foot(0); foot<4; foot++)
    {
        float contactState = contactStates[foot];
        float swingState = swingStates[foot];
        if(swingState > 0){ //foot is in swing
            if(firstSwing[foot])
            {
                firstSwing[foot] = false;
                footSwingTrajectory[foot].setInitialPosition(foot_p[foot]);
            }
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(swingState, timeSwing);
            swingTraj_p[foot] = footSwingTrajectory[foot].getPosition();
            swingTraj_v[foot] = footSwingTrajectory[foot].getVelocity();
            swingTraj_a[foot] = footSwingTrajectory[foot].getAcceleration();
        }
        else //foot is in stance
        {
            firstSwing[foot] = true;
            swingTraj_p[foot] = footSwingTrajectory[foot].getPosition();
            swingTraj_v[foot] = footSwingTrajectory[foot].getVelocity();
            swingTraj_a[foot] << 0,0,0;
        }
    }

    RecordData();

}

void FootHold::RecordData(){
    Vec31<float> path_p[100];
    Vec31<float> path_v[100];
    Vec31<float> path_a[100];

    int foot = 3;

    float step = (1.f - swingStates[foot])/100.f;
    for(int i=0; i<100; i++){
        footSwingTrajectory[foot].computeSwingTrajectoryBezier(swingStates[foot] + i*step, _gait->getCurrentSwingTime(_dt, 0));
        path_p[i] = footSwingTrajectory[foot].getPosition();
        path_v[i] = footSwingTrajectory[foot].getVelocity();
        path_a[i] = footSwingTrajectory[foot].getAcceleration();
    }

    ofstream in;
    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/SwingTrajectory_position.txt", ios::trunc);
    for(int i(0); i<100; i++){
        in << path_p[i][0] << "\t" << path_p[i][1] << "\t" << path_p[i][2] << "\n";
    }
    in.close();

    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/SwingTrajectory_velocity.txt", ios::trunc);
    for(int i(0); i<100; i++){
        in << path_v[i][0] << "\t" << path_v[i][1] << "\t" << path_v[i][2] << "\n";
    }
    in.close();

    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/SwingTrajectory_acceleration.txt", ios::trunc);
    for(int i(0); i<100; i++){
        in << path_a[i][0] << "\t" << path_a[i][1] << "\t" << path_a[i][2] << "\n";
    }
    in.close();
}
