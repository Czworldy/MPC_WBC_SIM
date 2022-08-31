#include "MotionPlanner.h"
#include "time.h"
#include "ros/ros.h"

using namespace std;

MotionPlanner::MotionPlanner(int nSegment, int iterationsBetweenSEG, double dt, 
                             UserParameter<float> & paramf,
                             UserParameter<double>& paramd):
    iterationCounter_(12*iterationsBetweenSEG),
    iterationsBetweenSEG_(iterationsBetweenSEG),
    nSpline_(nSegment),
    iter_gait_(0),
    iter_gait_pre_(0),
    dt_(dt),//Duration of one segment of Spline
    paramf_(paramf),
    paramd_(paramd),
    t_loop_(paramd.t_plan_control_loop),
    y_vel_des_(0.0),
    line_margin(paramf.line_margin),
    //walking_(nSpline_, Vec41<int>(4,14,19,9), Vec41<int>(16,16,16,16), "walking"),
    walking_(nSegment, Vec41<int>(36,16,36,16), Vec41<int>(24,24,24,24),"walking"),
    //walking_(nSegment, Vec41<int>(38,18,38,18), Vec41<int>(22,22,22,22),"walking"),
    //walking_(nSegment, Vec41<int>(34,14,34,14), Vec41<int>(26,26,26,26),"walking"),
    //walking_(nSegment, Vec41<int>(26,6,26,6), Vec41<int>(34,34,34,34),"walking"),
    //walking_(nSegment, Vec41<int>(28,8,28,8), Vec41<int>(32,32,32,32),"walking"),
    //walking_(nSegment, Vec41<int>(30,10,30,10), Vec41<int>(30,30,30,30),"walking"),
    //walking_(nSegment, Vec41<int>(32,12,32,12), Vec41<int>(28,28,28,28),"walking"),
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

    CoMSpline_.resize(12*nSpline_);
    CoMSpline_Pre_.resize(12*nSpline_);
}

MotionPlanner::~MotionPlanner(){
    delete[] P_Acc;
    delete[] P_Devia;
    delete[] P_Path;
    delete[] P_Final;
    delete[] P_;

    delete[] q_Devia;
    delete[] q_Path;
    delete[] q_Final;
    delete[] q_;
}

void MotionPlanner::initialize(){
    for(int i(0); i<4; i++)
        firstSwing[i] = true;
    firstRun = true;
}

void MotionPlanner::_SetupCommand(ControlFSMData<float>& data){

    // if((iter_gait_ - iter_gait_pre_ >= 1)&&(y_vel_des_ < data.userParameters.vel_cmd[1])){
    //     y_vel_des_ += 0.02;
    //     iter_gait_pre_ = iter_gait_;

    //     if(y_vel_des_ >= 0.15){
    //         line_margin = 0.006;
    //     }

    //     if(y_vel_des_ >= 0.17){
    //         line_margin = 0.007;
    //     }
    // }

    x_vel_des_ = data.userParameters.vel_cmd[0];
    yaw_turn_rate_ = data.userParameters.vel_cmd[2];

    yaw_des_ = data.bodyStateEst.rpy[2] + paramf_.t_plan_control_loop*yaw_turn_rate_;
    roll_des_ = 0;
    pitch_des_ = 0;
    body_height_ = data.userParameters.body_height;
}

void MotionPlanner::_UpdateModel(const BodyStateEstData<float> & bodyEst,                               
                                 const LegStateEstData<float> * legEst){
    
    state_.bodyOrientation.w() = bodyEst.orientation.w();
    state_.bodyOrientation.x() = bodyEst.orientation.x();
    state_.bodyOrientation.y() = bodyEst.orientation.y();
    state_.bodyOrientation.z() = bodyEst.orientation.z();
    for(size_t i(0); i<3; i++){
        state_.bodyPosition[i] = bodyEst.position[i];
        state_.bodyVelocity[i] = bodyEst.vBody[i];
        state_.bodyVelocity[i+3] = bodyEst.omegaBody[i];

        state_.q_leg[3*(legID::LF)+i] = legEst[legID_P::LF].q[i];
        state_.qd_leg[3*(legID::LF)+i] = legEst[legID_P::LF].qd[i];

        state_.q_leg[3*(legID::LB)+i] = legEst[legID_P::LB].q[i];
        state_.qd_leg[3*(legID::LB)+i] = legEst[legID_P::LB].qd[i];

        state_.q_leg[3*(legID::RB)+i] = legEst[legID_P::RB].q[i];
        state_.qd_leg[3*(legID::RB)+i] = legEst[legID_P::RB].qd[i];

        state_.q_leg[3*(legID::RF)+i] = legEst[legID_P::RF].q[i];
        state_.qd_leg[3*(legID::RF)+i] = legEst[legID_P::RF].qd[i];
    }

    state_.contact_state_[legID::LF] = bodyEst.contactEstimate_P[legID_P::LF];
    state_.contact_state_[legID::LB] = bodyEst.contactEstimate_P[legID_P::LB];
    state_.contact_state_[legID::RB] = bodyEst.contactEstimate_P[legID_P::RB];
    state_.contact_state_[legID::RF] = bodyEst.contactEstimate_P[legID_P::RF];

    jueying.setState(state_);
}

void MotionPlanner::run(ControlFSMData<float>& data, DesMotionData & result){


    _SetupCommand(data);
    _UpdateModel(data.bodyStateEst, data.legStateEst_P);
    walking_.setIterations(iterationsBetweenSEG_, iterationCounter_);

    //FOOTHOLDS AND SWING TRAJECTORY PLAN
    Vec2<float> vel_des(x_vel_des_, y_vel_des_);
    Vec31<float> hip_Pos[4];
    Vec2<float> hip_Vel[4];
    Vec31<float> footLocation[4];
    long long int index_gait(iterationCounter_%(iterationsBetweenSEG_*nSpline_));

    gaitTable_ = walking_.getGaitPatternMat();
    for(int i(0); i<4; i++){
        if(index_gait >= 16*iterationsBetweenSEG_ && index_gait < 36*iterationsBetweenSEG_){
            hip_Pos[i] = jueying.hipPosition_bias_lf_rb(i).cast<float>();
            hip_Vel[i] = jueying.hipVelocity_bias_lf_rb(i).cast<float>().head(2);
        }
        else{
            hip_Pos[i] = jueying.hipPosition_bias_lb_rf(i).cast<float>();
            hip_Vel[i] = jueying.hipVelocity_bias_lb_rf(i).cast<float>().head(2);  
        }

        footLocation[i] = jueying.footPosition(i).cast<float>();
        footStateCur_.isContact[i] = data.bodyStateEst.contactEstimate_P[i];
        footStateCur_.footLocation[i] =footLocation[i].head(2);
    }
    footHold_.FootHoldPlan(vel_des, hip_Vel, hip_Pos, footLocation, 
                            footHoldDes_, result.swingFoot_p_, result.swingFoot_v_, result.swingFoot_a_);

    //FOOTHOLD SPEACIAL
    // for(int i(0); i<4; i++){
    //     footHoldDes_[i][0] = footStateCur_.footLocation[i][0];
    // }

    result.iterCounter = iterationCounter_ % (nSpline_ * iterationsBetweenSEG_);
    result.footLocation_lf = footHoldDes_[0];
    result.footLocation_lb = footHoldDes_[1];
    result.footLocation_rb = footHoldDes_[2];
    result.footLocation_rf = footHoldDes_[3];
    result.contactStateIter[0] = gaitTable_(0,0);
    result.contactStateIter[1] = gaitTable_(1,0);
    result.contactStateIter[2] = gaitTable_(2,0);
    result.contactStateIter[3] = gaitTable_(3,0);

    // cout << "CONTACT STATE PLAN： " << endl;
    // cout << result.contactStateIter[0] << " " << result.contactStateIter[1] << " " << result.contactStateIter[2] << " " << result.contactStateIter[3] << endl;
    // cout << "SWING STATE PLAN: " << endl;
    // cout << walking_.getSwingState()[0] << " " << walking_.getSwingState()[1] << " " << walking_.getSwingState()[2] << " " << walking_.getSwingState()[3] << endl;

    //SUPPORT_POLYGON
    supportPolygon_.SearchPolygon(gaitTable_, footStateCur_, footHoldDes_, iterationCounter_, line_margin);

    // cout << "GAIT PATTERN: " << endl;
    // ROS_INFO_STREAM("\n" << gaitTable_);
 
    //COM MOTION PLAN
    if(firstRun){
        //AccelerationMin
        accelerationMin_.UpdateCostFunctionAcc(P_Acc);

        //SoftFinalConstraints && PathRegularization
        Vec2<float> pInit, vInit, aInit;
        Vec2<float> pFinal, vFinal, aFinal;

        pInit = jueying.get_CoM_Position().cast<float>().head(2);//data.bodyStateEst.position.head(2);//jueying.get_CoM_Position().cast<float>().head(2);
        vInit = jueying.get_CoM_Velocity().cast<float>().head(2);//data.bodyStateEst.vBody.head(2);//jueying.get_CoM_Velocity().cast<float>().head(2);
        aInit << 0.0, 0.0;
        pFinal =  pInit + dt_ * nSpline_ * vel_des;
        vFinal = vel_des;
        aFinal << 0.0, 0.0;

        // cout << "pInit: " << endl;
        // cout << pInit[0] << " " << pInit[1] << endl;
        // cout << "vInit: " << endl;
        // cout << vInit[0] << " " << vInit[1] << endl;
        // cout << "aInit: " << endl;
        // cout << aInit[0] << " " << aInit[1] << endl;
        // cout << "pFinal: " << endl;
        // cout << pFinal[0] << " " << pFinal[1] << endl;
        // cout << "vFinal: " << endl;
        // cout << vFinal[0] << " " << vFinal[1] << endl;
        // cout << "aFinal: " << endl;
        // cout << aFinal[0] << " " << aFinal[1] << endl;

        softFinalConstraints_.UpdateCostFunction(pFinal, P_Final, q_Final);
        pathRegularization_.UpdateCostFunction(pInit, vInit, aInit, pFinal, vFinal, aFinal, P_Path, q_Path);
        
        //RUN QP
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
        // cout << "if_planned: " << if_planned << endl; 
        // cout << "FIRST RUN TRUE" << endl;
    }
    else{
        //AccelerationMin
        accelerationMin_.UpdateCostFunctionAcc(P_Acc);

        //SoftFinalConstraints && PathRegularization
        Vec2<float> pInit, vInit, aInit;
        Vec2<float> pFinal, vFinal, aFinal;

        float alpha_p = 0.5*exp(-paramf_.lamada_p*t_loop_);
        float alpha_v = 0.5*exp(-paramf_.lamada_v*t_loop_);

        pInit = alpha_p*getCoMDesPosition(t_loop_) + (1.-alpha_p) * jueying.get_CoM_Position().cast<float>().head(2);
        vInit = alpha_v*getCoMDesVelocity(t_loop_) + (1.- alpha_v) * jueying.get_CoM_Velocity().cast<float>().head(2);
        //pInit  = jueying.get_CoM_Position().cast<float>().head(2);//data.bodyStateEst.position.head(2);//jueying.get_CoM_Position().cast<float>().head(2);
        //vInit  = jueying.get_CoM_Velocity().cast<float>().head(2);//data.bodyStateEst.vBody.head(2);//jueying.get_CoM_Velocity().cast<float>().head(2);
        aInit  = getCoMDesAcceleration(t_loop_);
        pFinal = supportPolygon_.getFinalPolygonCenter();
        vFinal = vel_des;
        aFinal << 0.0, 0.0;

        // cout << "pInit: " << endl;
        // cout << pInit[0] << " " << pInit[1] << endl;
        // cout << "vInit: " << endl;
        // cout << vInit[0] << " " << vInit[1] << endl;
        // cout << "aInit: " << endl;
        // cout << aInit[0] << " " << aInit[1] << endl;
        // cout << "pFinal: " << endl;
        // cout << pFinal[0] << " " << pFinal[1] << endl;
        // cout << "vFinal: " << endl;
        // cout << vFinal[0] << " " << vFinal[1] << endl;
        // cout << "aFinal: " << endl;
        // cout << aFinal[0] << " " << aFinal[1] << endl;



        softFinalConstraints_.UpdateCostFunction(pFinal, P_Final, q_Final);
        pathRegularization_.UpdateCostFunction(pInit, vInit, aInit, pFinal, vFinal, aFinal, P_Path, q_Path);

        //DeviationFromPre
        deviationFromPre_.UpdateCostFunction(CoMSpline_Pre_,P_Devia, q_Devia);

        //RUN QP
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
        // cout << "SECOND RUN TRUE" << endl;
        // cout << "if_planned: " << if_planned << endl; 
    }

    for(int i(0); i<4; i++){
        for(int j(0); j<6; j++){
            result.splineX[6*i+j] = CoMSpline_[12*i+j];
            result.splineY[6*i+j] = CoMSpline_[12*i+j+6];
        }
    }

    iterationCounter_++;
    if(!(iterationCounter_ % (nSpline_ * iterationsBetweenSEG_))){
        iter_gait_++;
    }
    
    _RecordData();

    //Print Data
    // cout << "hip_pos_lf: " << endl;
    // cout << hip_Pos[0][0] << " " << hip_Pos[0][1] << " " << hip_Pos[0][2] << endl;
    // cout << "hip_pos_lb: " << endl;
    // cout << hip_Pos[1][0] << " " << hip_Pos[1][1] << " " << hip_Pos[1][2] << endl;
    // cout << "hip_pos_rb: " << endl;
    // cout << hip_Pos[2][0] << " " << hip_Pos[2][1] << " " << hip_Pos[2][2] << endl;
    // cout << "hip_pos_rf: " << endl;
    // cout << hip_Pos[3][0] << " " << hip_Pos[3][1] << " " << hip_Pos[3][2] << endl;
    // cout << "\n" << endl;
    // cout << "hip_vel_lf: " << endl;
    // cout << hip_Vel[0][0] << " " << hip_Vel[0][1] << endl;
    // cout << "hip_vel_lb: " << endl;
    // cout << hip_Vel[1][0] << " " << hip_Vel[1][1] << endl;
    // cout << "hip_vel_rb: " << endl;
    // cout << hip_Vel[2][0] << " " << hip_Vel[2][1] << endl;
    // cout << "hip_vel_rf: " << endl;
    // cout << hip_Vel[3][0] << " " << hip_Vel[3][1] << endl;
    // cout << "\n" << endl;
    // cout << "foot_location_lf" << endl;
    // cout << footLocation[0][0] << " " << footLocation[0][1] << " " << footLocation[0][2] << endl;
    // cout << "foot_location_lb" << endl;
    // cout << footLocation[1][0] << " " << footLocation[1][1] << " " << footLocation[1][2] << endl;
    // cout << "foot_location_rb" << endl;
    // cout << footLocation[2][0] << " " << footLocation[2][1] << " " << footLocation[2][2] << endl;
    // cout << "foot_location_rf" << endl;
    // cout << footLocation[3][0] << " " << footLocation[3][1] << " " << footLocation[3][2] << endl;
    // cout << "\n" << endl;
    // cout << "foot isContact" << endl;
    // cout << footStateCur_.isContact[0] << " " << footStateCur_.isContact[1] << " " << footStateCur_.isContact[2] << " " << footStateCur_.isContact[3] << endl;
    // cout << "\n" << endl;
    // cout << "CoM Position: " << endl;
    // cout << jueying.get_CoM_Position().cast<float>()[0] << " " << jueying.get_CoM_Position().cast<float>()[1] << endl;
    // cout << "CoM Velocity: " << endl;
    // cout << jueying.get_CoM_Velocity().cast<float>()[0] << " " << jueying.get_CoM_Velocity().cast<float>()[1] << endl;

}

Vec2<float> MotionPlanner::getCoMDesPosition(double time){//time in a periodic

    Vec2<double> des_pos(0,0);
    if(time <= dt_){
        des_pos = searchCoMCoeff_._timePosMat(time) * CoMSpline_Pre_.segment(0, 12);
    }
    else{
        des_pos = searchCoMCoeff_._timePosMat(time) * CoMSpline_Pre_.tail(12);
    }
    return des_pos.cast<float>();
}

Vec2<float> MotionPlanner::getCoMDesVelocity(double time){//time in a periodic

    Vec2<double> des_vel(0,0);
    if(time <= dt_){
        des_vel = searchCoMCoeff_._timeVelMat(time) * CoMSpline_Pre_.segment(0, 12);
    }
    else{
        des_vel = searchCoMCoeff_._timeVelMat(time) * CoMSpline_Pre_.tail(12);
    }
    return des_vel.cast<float>();
}

Vec2<float> MotionPlanner::getCoMDesAcceleration(double time){//time in a periodic

    Vec2<double> des_acc(0,0);
    if(time <= dt_){
        des_acc = searchCoMCoeff_._timeAccMat(time) * CoMSpline_Pre_.segment(0, 12);
    }
    else{
        des_acc = searchCoMCoeff_._timeAccMat(time) * CoMSpline_Pre_.tail(12);
    }

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
