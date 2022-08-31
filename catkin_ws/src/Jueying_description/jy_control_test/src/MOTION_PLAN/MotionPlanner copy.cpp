#include "MotionPlanner.h"
#include "time.h"
#include "ros/ros.h"

MotionPlanner::MotionPlanner(int nSegment, int iterationsBetweenSEG, double dt, 
                            UserParameter<float> & paramf,
                            UserParameter<double>& paramd):

    iterationCounter_(14*iterationsBetweenSEG),
    iterationsBetweenSEG_(iterationsBetweenSEG),
    nSpline_(nSegment),
    dt_(dt),//Duration of one segment of Spline
    paramf_(paramf),
    paramd_(paramd),
    t_loop_(paramd.t_plan_control_loop),
    // walking_(nSpline_, Vec41<int>(4,14,19,9), Vec41<int>(16,16,16,16), "walking"),
    //walking_(nSegment, Vec41<int>(36,16,36,16), Vec41<int>(24,24,24,24),"walking"),
    //walking_(nSegment, Vec41<int>(26,6,26,6), Vec41<int>(34,34,34,34),"walking"),
    walking_(nSegment, Vec41<int>(28,8,28,8), Vec41<int>(32,32,32,32),"walking"),
    //walking_(nSegment, Vec41<int>(30,10,30,10), Vec41<int>(30,30,30,30),"walking"),
    footHold_(&walking_, dt, paramf_),
    supportPolygon_(nSegment, iterationsBetweenSEG),
    accelerationMin_(nSegment, iterationsBetweenSEG, dt),
    deviationFromPre_(nSegment, iterationsBetweenSEG, dt, paramd),
    pathRegularization_(nSegment, iterationsBetweenSEG, dt, paramd),
    softFinalConstraints_(nSegment, iterationsBetweenSEG, dt, paramd),
    searchCoMCoeff_(nSegment, iterationsBetweenSEG, dt, &supportPolygon_, paramd){

    for(int i=0; i<4; i++)
        firstSwing[i] =true;
    size_t dimVar(12*nSpline_);

    P_Acc = new c_float[42*nSpline_]; 
    P_Devia = new c_float[42*nSpline_]; 
    P_Path = new c_float[42*nSpline_]; 
    P_Final = new c_float[42*nSpline_]; 
    P_ = new c_float[42*nSpline_]; 

    q_Devia = new c_float[dimVar];
    q_Path = new c_float[dimVar];
    q_Final = new c_float[dimVar];
    q_ = new c_float[dimVar];
}

MotionPlanner::~MotionPlanner(){
    delete [] P_Acc;
    delete [] P_Devia;
    delete [] P_Path;
    delete [] P_Final;
    delete [] P_;

    delete [] q_Devia;
    delete [] q_Path;
    delete [] q_Final;
    delete [] q_;
}

void MotionPlanner::initialize(){
    for(int i(0); i<4; i++)
        firstSwing[i] = true;
    firstRun = true;
}

void MotionPlanner::_SetupCommand(ControlFSMData<float>& data){
    x_vel_des_ = data.userParameters.vel_cmd[0];
    y_vel_des_ = data.userParameters.vel_cmd[1];
    yaw_turn_rate_ = data.userParameters.vel_cmd[2];

    yaw_des_ = data.bodyStateEst.rpy[2] + paramf_.t_plan_control_loop*yaw_turn_rate_;
    roll_des_ = 0;
    pitch_des_ = 0;
    body_height_ = data.userParameters.body_height;
}

void MotionPlanner::run(ControlFSMData<float>& data, DesMotionData & result){

    //Command Setup
    _SetupCommand(data);

    walking_.setIterations(iterationsBetweenSEG_, iterationCounter_);

    Vec2<float> vel_des(x_vel_des_, y_vel_des_);

    Vec31<float> hip_Pos[4];
    Vec31<float> hip_Vel3[4];
    Vec2<float> hip_Vel2[4];
    Vec31<float> body_vel_cur, body_omega_cur, body_pos_cur;
    Mat3<float> rBody;

    rBody = data.bodyStateEst.RotationMat;
    body_vel_cur = data.bodyStateEst.vBody;
    body_omega_cur = data.bodyStateEst.omegaBody;
    body_pos_cur = data.bodyStateEst.position;
    for(int i(0); i<4; i++){
        hip_Pos[i] = body_pos_cur + rBody * data.quadruped.hipLocation[i];
        hip_Vel3[i] = body_vel_cur + body_omega_cur.cross(rBody * data.quadruped.hipLocation[i]);
        hip_Vel2[i] = hip_Vel3[i].head(2);
    }

    //FOOTHOLDS AND SWING TRAJECTORY PLAN
    Vec31<float> footLocation3[4];
    for(int i(0); i<4; i++){
        footLocation3[i] =  (body_pos_cur + rBody * 
                            data.quadruped.getFootLocation(data.legStateEst_P[i].q, i));
        footStateCur_.isContact[i] = data.bodyStateEst.contactEstimate_P[i];
        footStateCur_.footLocation[i] =footLocation3[i].head(2);
    }
    
    footHold_.FootHoldPlan(vel_des, hip_Vel2, hip_Pos, footLocation3, 
                            footHoldDes_, result.swingFoot_p_, result.swingFoot_v_, result.swingFoot_a_);

    result.contactStateIter = walking_.getContactState();     

    //SUPPORT_POLYGON
    gaitTable_ = walking_.getGaitPatternMat();
    supportPolygon_.SearchPolygon(gaitTable_, footStateCur_, footHoldDes_, iterationCounter_);

    //COM MOTION PLAN
    if(firstRun){
        //AccelerationMin
        accelerationMin_.UpdateCostFunctionAcc(P_Acc);
        //SoftFinalConstraints
        Vec2<float> pInit, vInit(0,0), aInit(0,0);
        Vec2<float> vFinal, aFinal;
        pInit = body_pos_cur.head(2); 
        Vec2<float> pFinal(pInit+dt_*nSpline_*vel_des);
        softFinalConstraints_.UpdateCostFunction(pFinal, P_Final, q_Final);
        //PathRegularization
        vFinal = vel_des;//参数待调
        aFinal << 0, 0;//参数待调
        pathRegularization_.UpdateCostFunction(pInit, vInit, aInit, pFinal, vFinal, aFinal,
                                               P_Path, q_Path);
                                
        for(int i(0); i<42*nSpline_; i++ ){
            P_[i] = P_Acc[i] + P_Path[i];
        }
        for(int i(0); i<42; i++){
            P_[42*nSpline_-42+i] += P_Final[i];
        }
        for(int i(0); i<12*nSpline_; i++){
            q_[i] = q_Path[i];
        }
        for(int i(0); i<12; i++){
            q_[12*nSpline_ - 12 + i] += q_Final[i];
        }
        searchCoMCoeff_.run(P_, q_, pInit, vInit, aInit, body_height_, pFinal, CoMSpline_, if_planned);

        if(if_planned){
            CoMSpline_Pre_ = CoMSpline_;
            firstRun = false;
            result.if_solved = true;
        }
        else{
            firstRun = true;
            result.if_solved = false;
        }

    }
    else{
        clock_t startTime, endTime;
        //AccelerationMin
        accelerationMin_.UpdateCostFunctionAcc(P_Acc);

        //SoftFinalConstraints
        //Vec2<float> pFinal(supportPolygon_.getFinalPolygonCenter());
        Vec2<float> pInit, vInit, aInit;
        Vec2<float> vFinal, aFinal;
        float alpha_p = 0.5*exp(-paramf_.lamada_p*t_loop_);//20210416:dt-->paramd.t_plan_control_loop
        float alpha_v = 0.5*exp(-paramf_.lamada_v*t_loop_);

        // pInit = alpha_p*getCoMDesPosition(t_loop_) + (1.-alpha_p)*body_pos_cur.head(2);
        // vInit = alpha_v*getCoMDesVelocity(t_loop_) + (1.- alpha_v)*body_vel_cur.head(2);
        // aInit = getCoMDesAcceleration(t_loop_);

        pInit = body_pos_cur.head(2);
        vInit = body_vel_cur.head(2);
        aInit = getCoMDesAcceleration(t_loop_);

        vFinal = getCoMDesVelocity(t_loop_);
        aFinal = getCoMDesAcceleration(t_loop_);
        Vec2<float> pFinal(pInit+dt_*nSpline_*vel_des);
        softFinalConstraints_.UpdateCostFunction(pFinal, P_Final, q_Final);

        //DeviationFromPre
        deviationFromPre_.UpdateCostFunction(CoMSpline_Pre_, 
                                             P_Devia, q_Devia);

        //PathRegularization
        pathRegularization_.UpdateCostFunction(pInit, vInit, aInit, pFinal, vFinal, aFinal,
                                               P_Path, q_Path);

        for(int i(0); i<42*nSpline_; i++ ){
            P_[i] = P_Acc[i] + P_Devia[i] + P_Path[i];
        }
        for(int i(0); i<42; i++){
            P_[42*nSpline_-42+i] += P_Final[i];
        }
        for(int i(0); i<12*nSpline_; i++){
            q_[i] = q_Devia[i] + q_Path[i];
        }
        for(int i(0); i<12; i++){
            q_[12*nSpline_ - 12 + i] += q_Final[i];
        }

        searchCoMCoeff_.run(P_, q_, pInit, vInit, aInit, body_height_, pFinal, CoMSpline_, if_planned);
        if(if_planned){
            CoMSpline_Pre_ = CoMSpline_;
            result.if_solved = true;
        }
        else{
            result.if_solved = false;
            firstRun = true;
        }
    }
//}
    for(int i(0); i<4; i++){
        for(int j(0); j<6; j++){
            result.splineX[6*i+j] = CoMSpline_[12*i+j];
            result.splineY[6*i+j] = CoMSpline_[12*i+j+6];
        }
    }

    iterationCounter_++;
    _RecordData();
}

Vec2<float> MotionPlanner::getCoMDesPosition(double time){//time in a periodic

    Vec2<double> des_pos(0,0);
    des_pos = searchCoMCoeff_._timePosMat(time) * CoMSpline_Pre_.segment(0, 12);   
    return des_pos.cast<float>();
}

Vec2<float> MotionPlanner::getCoMDesVelocity(double time){//time in a periodic

    Vec2<double> des_vel(0,0);
    des_vel = searchCoMCoeff_._timeVelMat(time) * CoMSpline_Pre_.segment(0, 12);
    return des_vel.cast<float>();
}

Vec2<float> MotionPlanner::getCoMDesAcceleration(double time){//time in a periodic

    Vec2<double> des_acc(0,0);
    des_acc = searchCoMCoeff_._timeAccMat(time) * CoMSpline_Pre_.segment(0, 12);
    return des_acc.cast<float>();
}

void MotionPlanner::_RecordData(){
    DMat<double> spline_xy(6*nSpline_, 2);
    DVec<double> spline_p(42*nSpline_);
    DVec<double> spline_final(42);


    for(int i(0); i<nSpline_; i++){
        spline_xy.block(6*i, 0, 6, 1) = CoMSpline_.segment(12*i, 6); 
        spline_xy.block(6*i, 1, 6, 1) = CoMSpline_.segment(12*i+6, 6); 
    }

    for(int i(0); i<42*nSpline_; i++){
        spline_p[i] = P_[i];
    }

    for(int i(0); i<42; i++){
        spline_final[i] = P_Final[i];
    }

    ofstream in;
    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/CoMMotion.txt", ios::trunc);
    for(int i(0); i<6*nSpline_; i++)
        in << spline_xy(i,0) << "\t" << spline_xy(i,1) <<"\n";
    in.close();

    ofstream in_p;
    in_p.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/OSQP_P.txt", ios::trunc);
    for(int i(0); i<42*nSpline_; i++){
        in_p << spline_p[i] <<"\n";
    }
    in_p.close();

    ofstream in_final;
    in_final.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/OSQP_FINAL.txt", ios::trunc);
    for(int i(0); i<42; i++){
        in_final << spline_final[i] <<"\n";
    }
    in_final.close();
}
