#include "simpleMotion.h"
#include <chrono>

SimpleMotion::SimpleMotion(bool verbose):
isPDSetUp(false),
isWBCSetUp(false),
isWBCSwingSetUp(false),
verbose_(verbose) {
    wbc_ctrl_.reset(new WBC_Ctrl<float>(&jueying_));
    timeCycle_ = paramf.cycle_time;

    // in_x.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_x.txt", ios::trunc);
    // in_y.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_y.txt", ios::trunc);
    // in_z.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_z.txt", ios::trunc);
    // in_roll.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_roll.txt", ios::trunc);
    // in_pitch.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_pitch.txt", ios::trunc);
    // in_yaw.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_yaw.txt", ios::trunc);

    // in_x_vel.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_x_vel.txt", ios::trunc);
    // in_y_vel.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_y_vel.txt", ios::trunc);
    // in_z_vel.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_z_vel.txt", ios::trunc);
    // in_roll_vel.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_roll_vel.txt", ios::trunc);
    // in_pitch_vel.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_pitch_vel.txt", ios::trunc);
    // in_yaw_vel.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_yaw_vel.txt", ios::trunc);

    // in_foot_lf_x.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_x.txt", ios::trunc);
    // in_foot_lf_y.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_y.txt", ios::trunc);
    // in_foot_lf_z.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_z.txt", ios::trunc);

    // in_foot_rf_x.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_x.txt", ios::trunc);
    // in_foot_rf_y.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_y.txt", ios::trunc);
    // in_foot_rf_z.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_z.txt", ios::trunc);

    // in_foot_lh_x.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_x.txt", ios::trunc);
    // in_foot_lh_y.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_y.txt", ios::trunc);
    // in_foot_lh_z.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_z.txt", ios::trunc);

    // in_foot_rh_x.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_x.txt", ios::trunc);
    // in_foot_rh_y.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_y.txt", ios::trunc);
    // in_foot_rh_z.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_z.txt", ios::trunc);

    // in_foot_joint_lf.open("/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/in_foot_joint_lf.txt", ios::trunc);
}

void SimpleMotion::EstimatedStatesInput(const EstimatorOutput& input) {
    // for PD
    for(int i(0); i < 3; i++) {
        currentLimbsStates_.lf_pos.value[i] = input.jointStates.lf_pos.value[i];
        currentLimbsStates_.rf_pos.value[i] = input.jointStates.rf_pos.value[i];
        currentLimbsStates_.lh_pos.value[i] = input.jointStates.lh_pos.value[i];
        currentLimbsStates_.rh_pos.value[i] = input.jointStates.rh_pos.value[i];

        currentLimbsStates_.lf_vel.value[i] = input.jointStates.lf_vel.value[i];
        currentLimbsStates_.rf_vel.value[i] = input.jointStates.rf_vel.value[i];
        currentLimbsStates_.lh_vel.value[i] = input.jointStates.lh_vel.value[i];
        currentLimbsStates_.rh_vel.value[i] = input.jointStates.rh_vel.value[i];
    }
    // for WBC
    currentStatesWBC_.bodyStateEst.base_pos_world << input.base_pos_world[0], input.base_pos_world[1], input.base_pos_world[2];
    currentStatesWBC_.bodyStateEst.base_orientation_world.w() = input.base_orientation_world.w();
    currentStatesWBC_.bodyStateEst.base_orientation_world.x() = input.base_orientation_world.x();
    currentStatesWBC_.bodyStateEst.base_orientation_world.y() = input.base_orientation_world.y();
    currentStatesWBC_.bodyStateEst.base_orientation_world.z() = input.base_orientation_world.z();
    currentStatesWBC_.bodyStateEst.base_rpy_world = quaternionTOrpy(currentStatesWBC_.bodyStateEst.base_orientation_world);
    currentStatesWBC_.bodyStateEst.base_rotMat_world = rpyTORotateMat(currentStatesWBC_.bodyStateEst.base_rpy_world[0],
                                                                      currentStatesWBC_.bodyStateEst.base_rpy_world[1],
                                                                      currentStatesWBC_.bodyStateEst.base_rpy_world[2]);
    
    currentStatesWBC_.bodyStateEst.base_linear_vel_world << input.base_linear_vel_world[0], input.base_linear_vel_world[1], input.base_linear_vel_world[2];
    currentStatesWBC_.bodyStateEst.base_linear_vel_body << input.base_linear_vel_body[0], input.base_linear_vel_body[1], input.base_linear_vel_body[2];
    currentStatesWBC_.bodyStateEst.base_angular_vel_world << input.base_angular_vel_world[0], input.base_angular_vel_world[1], input.base_angular_vel_world[2];
    currentStatesWBC_.bodyStateEst.base_angular_vel_body << input.base_angular_vel_body[0], input.base_angular_vel_body[1], input.base_angular_vel_body[2];

    for (int i(0); i < 3; i++) {
        currentStatesWBC_.legStateEst[legID::LF].q[i] = input.jointStates.lf_pos.value[i];
        currentStatesWBC_.legStateEst[legID::RF].q[i] = input.jointStates.rf_pos.value[i];
        currentStatesWBC_.legStateEst[legID::LB].q[i] = input.jointStates.lh_pos.value[i];
        currentStatesWBC_.legStateEst[legID::RB].q[i] = input.jointStates.rh_pos.value[i];

        currentStatesWBC_.legStateEst[legID::LF].qd[i] = input.jointStates.lf_vel.value[i];
        currentStatesWBC_.legStateEst[legID::RF].qd[i] = input.jointStates.rf_vel.value[i];
        currentStatesWBC_.legStateEst[legID::LB].qd[i] = input.jointStates.lh_vel.value[i];
        currentStatesWBC_.legStateEst[legID::RB].qd[i] = input.jointStates.rh_vel.value[i];
    }
}

void SimpleMotion::PDSetUpInitialLimbsAngles() {
    for(int i(0); i < 3; i++) {
        initialLimbsAngles_.lf_pos.value[i] = currentLimbsStates_.lf_pos.value[i];
        initialLimbsAngles_.rf_pos.value[i] = currentLimbsStates_.rf_pos.value[i];
        initialLimbsAngles_.lh_pos.value[i] = currentLimbsStates_.lh_pos.value[i];
        initialLimbsAngles_.rh_pos.value[i] = currentLimbsStates_.rh_pos.value[i];
    }

    if(verbose_) {
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpInitialLimbsAngles] Done!";
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpInitialLimbsAngles] initial_angle_lf: " << initialLimbsAngles_.lf_pos.value[0] << " " << initialLimbsAngles_.lf_pos.value[1] << " " << initialLimbsAngles_.lf_pos.value[2];
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpInitialLimbsAngles] initial_angle_rf: " << initialLimbsAngles_.rf_pos.value[0] << " " << initialLimbsAngles_.rf_pos.value[1] << " " << initialLimbsAngles_.rf_pos.value[2];
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpInitialLimbsAngles] initial_angle_lh: " << initialLimbsAngles_.lh_pos.value[0] << " " << initialLimbsAngles_.lh_pos.value[1] << " " << initialLimbsAngles_.lh_pos.value[2];
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpInitialLimbsAngles] initial_angle_rh: " << initialLimbsAngles_.rh_pos.value[0] << " " << initialLimbsAngles_.rh_pos.value[1] << " " << initialLimbsAngles_.rh_pos.value[2];
    }
}

void SimpleMotion::PDSetUpGoalLimbsAngles(double angle_haa, double angle_hfe, double angle_kfe) {
    goalLimbsAngles_.lf_pos.value[0] = angle_haa; goalLimbsAngles_.lf_pos.value[1] = angle_hfe; goalLimbsAngles_.lf_pos.value[2] = angle_kfe;
    goalLimbsAngles_.rf_pos.value[0] = angle_haa; goalLimbsAngles_.rf_pos.value[1] = angle_hfe; goalLimbsAngles_.rf_pos.value[2] = angle_kfe;
    goalLimbsAngles_.lh_pos.value[0] = angle_haa; goalLimbsAngles_.lh_pos.value[1] = angle_hfe; goalLimbsAngles_.lh_pos.value[2] = angle_kfe;
    goalLimbsAngles_.rh_pos.value[0] = angle_haa; goalLimbsAngles_.rh_pos.value[1] = angle_hfe; goalLimbsAngles_.rh_pos.value[2] = angle_kfe;
}

void SimpleMotion::PDSetUpGoalTime(double timeGoal) {
    timeGoalPD_ = timeGoal;
}

bool SimpleMotion::PDSetUpMotion(double angle_haa, double angle_hfe, double angle_kfe, double timeGoal) {
    PDSetUpInitialLimbsAngles();
    PDSetUpGoalLimbsAngles(angle_haa, angle_hfe, angle_kfe);
    PDSetUpGoalTime(timeGoal);
    iteratorPD_ = 0;
    timeNowPD_ = 0;

    isPDSetUp = true;

    if(verbose_) {
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpMotion] Done!";
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpMotion] angle_haa_Goal: " << angle_haa;
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpMotion] angle_hfe_Goal: " << angle_hfe;
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpMotion] angle_kfe_Goal: " << angle_kfe;
        std::cerr << "\n[yjy: SimpleMotion::PDSetUpMotion] timeGoal: " << timeGoal;
    }

    return true;
}

void SimpleMotion::CubicSplinePlanForLimbs() {
    Vec31<double> lf_Traj[3];
    Vec31<double> rf_Traj[3];
    Vec31<double> lh_Traj[3];
    Vec31<double> rh_Traj[3]; 

    for (int i(0); i < 3; i++) {
        lf_Traj[i] = TrajectoryPlan_d(initialLimbsAngles_.lf_pos.value[i], goalLimbsAngles_.lf_pos.value[i], timeGoalPD_, timeNowPD_);
        rf_Traj[i] = TrajectoryPlan_d(initialLimbsAngles_.rf_pos.value[i], goalLimbsAngles_.rf_pos.value[i], timeGoalPD_, timeNowPD_);
        lh_Traj[i] = TrajectoryPlan_d(initialLimbsAngles_.lh_pos.value[i], goalLimbsAngles_.lh_pos.value[i], timeGoalPD_, timeNowPD_);
        rh_Traj[i] = TrajectoryPlan_d(initialLimbsAngles_.rh_pos.value[i], goalLimbsAngles_.rh_pos.value[i], timeGoalPD_, timeNowPD_);
    }

    for (int i(0); i < 3; i++) {
        planedLimbsStates_.lf_pos.value[i] = lf_Traj[i][0];
        planedLimbsStates_.rf_pos.value[i] = rf_Traj[i][0];
        planedLimbsStates_.lh_pos.value[i] = lh_Traj[i][0];
        planedLimbsStates_.rh_pos.value[i] = rh_Traj[i][0];

        planedLimbsStates_.lf_vel.value[i] = lf_Traj[i][1];
        planedLimbsStates_.rf_vel.value[i] = rf_Traj[i][1];
        planedLimbsStates_.lh_vel.value[i] = lh_Traj[i][1];
        planedLimbsStates_.rh_vel.value[i] = rh_Traj[i][1];
    }
}

void SimpleMotion::KeepLimbsAngles() {
    for (int i(0); i < 3; i++) {
        planedLimbsStates_.lf_pos.value[i] = goalLimbsAngles_.lf_pos.value[i];
        planedLimbsStates_.rf_pos.value[i] = goalLimbsAngles_.rf_pos.value[i];
        planedLimbsStates_.lh_pos.value[i] = goalLimbsAngles_.lh_pos.value[i];
        planedLimbsStates_.rh_pos.value[i] = goalLimbsAngles_.rh_pos.value[i];

        planedLimbsStates_.lf_vel.value[i] = 0.0;
        planedLimbsStates_.rf_vel.value[i] = 0.0;
        planedLimbsStates_.lh_vel.value[i] = 0.0;
        planedLimbsStates_.rh_vel.value[i] = 0.0;
    }   
}

Vec31<double> SimpleMotion::TrajectoryPlan_d(double startPoint, double finalPoint, double finalTime, double time_traj){
    double a_0, a_1, a_2, a_3;
    Vec31<double> point_inter;
    a_0 = startPoint;
	a_1 = 0;
	a_2 = 3* (finalPoint - startPoint)/(pow(finalTime,2));
	a_3 = -2* (finalPoint - startPoint)/(powf(finalTime,3));

	point_inter[0] = a_0 + a_1 * time_traj + a_2 *pow(time_traj,2) + a_3 * pow(time_traj,3);//p
	point_inter[1] = a_1 + a_2 * 2 * time_traj + a_3 * 3 * pow(time_traj,2);//v
	point_inter[2] = a_2 * 2 + a_3 * 3 * 2 * time_traj;//a

    return point_inter;
}

bool SimpleMotion::isPDMotionFinished() {
    if(timeNowPD_ <= timeGoalPD_) {
        return false;
    }
    else {
        return true;
    }
}

void SimpleMotion::PDMotionRun(LimbsCommand& command) {

    if(!isPDSetUp) {
        std::cerr << "\n[yjy]: SimpleMotion::PDMotionRun]:YOU SHOULD SETUP FIRST!";
        
        for (int i(0); i < 3; i++) {
            command.lf_tau.value[i] = 0;
            command.rf_tau.value[i] = 0;
            command.lh_tau.value[i] = 0;
            command.rh_tau.value[i] = 0;
        }
        return;
    }

    if(!isPDMotionFinished()) {
        timeNowPD_ = iteratorPD_ * timeCycle_;
        CubicSplinePlanForLimbs();
    }
    else {
        KeepLimbsAngles();
    }
    
    iteratorPD_++;

    command.lf_tau.value[0] = paramf.Kp_lf_haa_pd * (planedLimbsStates_.lf_pos.value[0] - currentLimbsStates_.lf_pos.value[0]) + paramf.Kd_lf_haa_pd * (planedLimbsStates_.lf_vel.value[0] - currentLimbsStates_.lf_vel.value[0]); // LF LH RF RH
    command.lf_tau.value[1] = paramf.Kp_lf_hfe_pd * (planedLimbsStates_.lf_pos.value[1] - currentLimbsStates_.lf_pos.value[1]) + paramf.Kd_lf_hfe_pd * (planedLimbsStates_.lf_vel.value[1] - currentLimbsStates_.lf_vel.value[1]); 
    command.lf_tau.value[2] = paramf.Kp_lf_kfe_pd * (planedLimbsStates_.lf_pos.value[2] - currentLimbsStates_.lf_pos.value[2]) + paramf.Kd_lf_kfe_pd * (planedLimbsStates_.lf_vel.value[2] - currentLimbsStates_.lf_vel.value[2]); 

    command.rf_tau.value[0] = paramf.Kp_rf_haa_pd * (planedLimbsStates_.rf_pos.value[0] - currentLimbsStates_.rf_pos.value[0]) + paramf.Kd_rf_haa_pd * (planedLimbsStates_.rf_vel.value[0] - currentLimbsStates_.rf_vel.value[0]); // LF LH RF RH
    command.rf_tau.value[1] = paramf.Kp_rf_hfe_pd * (planedLimbsStates_.rf_pos.value[1] - currentLimbsStates_.rf_pos.value[1]) + paramf.Kd_rf_hfe_pd * (planedLimbsStates_.rf_vel.value[1] - currentLimbsStates_.rf_vel.value[1]); 
    command.rf_tau.value[2] = paramf.Kp_rf_kfe_pd * (planedLimbsStates_.rf_pos.value[2] - currentLimbsStates_.rf_pos.value[2]) + paramf.Kd_rf_kfe_pd * (planedLimbsStates_.rf_vel.value[2] - currentLimbsStates_.rf_vel.value[2]); 

    command.lh_tau.value[0] = paramf.Kp_lh_haa_pd * (planedLimbsStates_.lh_pos.value[0] - currentLimbsStates_.lh_pos.value[0]) + paramf.Kd_lh_haa_pd * (planedLimbsStates_.lh_vel.value[0] - currentLimbsStates_.lh_vel.value[0]); // LF LH RF RH
    command.lh_tau.value[1] = paramf.Kp_lh_hfe_pd * (planedLimbsStates_.lh_pos.value[1] - currentLimbsStates_.lh_pos.value[1]) + paramf.Kd_lh_hfe_pd * (planedLimbsStates_.lh_vel.value[1] - currentLimbsStates_.lh_vel.value[1]); 
    command.lh_tau.value[2] = paramf.Kp_lh_kfe_pd * (planedLimbsStates_.lh_pos.value[2] - currentLimbsStates_.lh_pos.value[2]) + paramf.Kd_lh_kfe_pd * (planedLimbsStates_.lh_vel.value[2] - currentLimbsStates_.lh_vel.value[2]); 

    command.rh_tau.value[0] = paramf.Kp_rh_haa_pd * (planedLimbsStates_.rh_pos.value[0] - currentLimbsStates_.rh_pos.value[0]) + paramf.Kd_rh_haa_pd * (planedLimbsStates_.rh_vel.value[0] - currentLimbsStates_.rh_vel.value[0]); // LF LH RF RH
    command.rh_tau.value[1] = paramf.Kp_rh_hfe_pd * (planedLimbsStates_.rh_pos.value[1] - currentLimbsStates_.rh_pos.value[1]) + paramf.Kd_rh_hfe_pd * (planedLimbsStates_.rh_vel.value[1] - currentLimbsStates_.rh_vel.value[1]); 
    command.rh_tau.value[2] = paramf.Kp_rh_kfe_pd * (planedLimbsStates_.rh_pos.value[2] - currentLimbsStates_.rh_pos.value[2]) + paramf.Kd_rh_kfe_pd * (planedLimbsStates_.rh_vel.value[2] - currentLimbsStates_.rh_vel.value[2]); 

    if(verbose_) {
        std::cerr << "\n[yjy: SimpleMotion::PDMotionRun] timePD_Now: " << timeNowPD_;
        std::cerr << "\n[yjy: SimpleMotion::PDMotionRun] timeCycle: " << timeCycle_;
        std::cerr << "\n[yjy: SimpleMotion::PDMotionRun] paramf.Kp_lf_haa_pd: " <<  paramf.Kp_lf_haa_pd;
        std::cerr << "\n[yjy: SimpleMotion::PDMotionRun] planedLimbsStates_.lf_pos.value[0]: " <<  planedLimbsStates_.lf_pos.value[0];
        std::cerr << "\n[yjy: SimpleMotion::PDMotionRun] currentLimbsStates_.lf_pos.value[0]: " <<  currentLimbsStates_.lf_pos.value[0];
    }
}

void SimpleMotion::WBCSetUpInitialBaseStates() {
    initialBaseStates_.x[0] = currentStatesWBC_.bodyStateEst.base_pos_world[0];
    initialBaseStates_.y[0] = currentStatesWBC_.bodyStateEst.base_pos_world[1];
    initialBaseStates_.z[0] = currentStatesWBC_.bodyStateEst.base_pos_world[2];
    initialBaseStates_.roll[0]  = currentStatesWBC_.bodyStateEst.base_rpy_world[0];
    initialBaseStates_.pitch[0] = currentStatesWBC_.bodyStateEst.base_rpy_world[1];
    initialBaseStates_.yaw[0]   = currentStatesWBC_.bodyStateEst.base_rpy_world[2];
}

void SimpleMotion::WBCSetUpContactForBaseMotion() {
    currentStatesWBC_.bodyStateEst.contactEstimate << 1, 1, 1, 1;
    desiredDataWBC_.contact_state << 1, 1, 1, 1;
}

void SimpleMotion::WBCSetUpGoalBaseStates(float x, float y, float z, float roll, float pitch, float yaw) {
    goalBaseStates_.x[0] = currentStatesWBC_.bodyStateEst.base_pos_world[0] + x;
    goalBaseStates_.y[0] = currentStatesWBC_.bodyStateEst.base_pos_world[1] + y;
    goalBaseStates_.z[0] = currentStatesWBC_.bodyStateEst.base_pos_world[2] + z;
    goalBaseStates_.roll[0]  = currentStatesWBC_.bodyStateEst.base_rpy_world[0] + roll;
    goalBaseStates_.pitch[0] = currentStatesWBC_.bodyStateEst.base_rpy_world[1] + pitch;
    goalBaseStates_.yaw[0]   = currentStatesWBC_.bodyStateEst.base_rpy_world[2] + yaw;

    // goalBaseStates_.x[0] = 0;
    // goalBaseStates_.y[0] = 0;
    // goalBaseStates_.z[0] = 0.48;
    // goalBaseStates_.roll[0]  = 0;
    // goalBaseStates_.pitch[0] = 0;
    // goalBaseStates_.yaw[0]   = 0;
}

void SimpleMotion::WBCSetUpGoalTime(float timeGoal) {
    timeGoalWBC_ = timeGoal;

    std::cerr << "\n[yjy: SimpleMotion::WBCSetUpGoalTime] timeGoalWBC_: "   << timeGoalWBC_ << "\n";
}

void SimpleMotion::CubicSplinePlanForBase() {
    planedBaseStates_.x     = TrajectoryPlan_f(initialBaseStates_.x[0], goalBaseStates_.x[0], timeGoalWBC_, timeNowWBC_);
    planedBaseStates_.y     = TrajectoryPlan_f(initialBaseStates_.y[0], goalBaseStates_.y[0], timeGoalWBC_, timeNowWBC_);
    planedBaseStates_.z     = TrajectoryPlan_f(initialBaseStates_.z[0], goalBaseStates_.z[0], timeGoalWBC_, timeNowWBC_);
    planedBaseStates_.roll  = TrajectoryPlan_f(initialBaseStates_.roll[0], goalBaseStates_.roll[0], timeGoalWBC_, timeNowWBC_);
    planedBaseStates_.pitch = TrajectoryPlan_f(initialBaseStates_.pitch[0], goalBaseStates_.pitch[0], timeGoalWBC_, timeNowWBC_);
    planedBaseStates_.yaw   = TrajectoryPlan_f(initialBaseStates_.yaw[0], goalBaseStates_.yaw[0], timeGoalWBC_, timeNowWBC_);
}

Vec31<float> SimpleMotion::TrajectoryPlan_f(float startPoint, float finalPoint, float finalTime, float time_traj) {
    float a_0, a_1, a_2, a_3;
    Vec31<float> point_inter;
    a_0 = startPoint;
	a_1 = 0;
	a_2 = 3* (finalPoint - startPoint)/(pow(finalTime,2));
	a_3 = -2* (finalPoint - startPoint)/(powf(finalTime,3));

	point_inter[0] = a_0 + a_1 * time_traj + a_2 *pow(time_traj,2) + a_3 * pow(time_traj,3);//p
	point_inter[1] = a_1 + a_2 * 2 * time_traj + a_3 * 3 * pow(time_traj,2);//v
	point_inter[2] = a_2 * 2 + a_3 * 3 * 2 * time_traj;//a

    return point_inter;
}

bool SimpleMotion::WBCSetUpBaseMotion(float x, float y, float z, float roll, float pitch, float yaw, float timeGoal) {
    WBCSetUpInitialBaseStates();
    WBCSetUpContactForBaseMotion();
    WBCSetUpGoalBaseStates(x, y, z, roll, pitch, yaw);
    WBCSetUpGoalTime(timeGoal);
    iteratorWBC_ = 0;  
    timeNowWBC_ = 0;   

    isWBCSetUp = true;
    return true; 
}

void SimpleMotion::KeepBaseStates() {
    planedBaseStates_.x[0]     = goalBaseStates_.x[0]; 
    planedBaseStates_.y[0]     = goalBaseStates_.y[0]; 
    planedBaseStates_.z[0]     = goalBaseStates_.z[0]; 
    planedBaseStates_.roll[0]  = goalBaseStates_.roll[0]; 
    planedBaseStates_.pitch[0] = goalBaseStates_.pitch[0]; 
    planedBaseStates_.yaw[0]   = goalBaseStates_.yaw[0]; 

    planedBaseStates_.x[1]     = 0.0; 
    planedBaseStates_.y[1]     = 0.0; 
    planedBaseStates_.z[1]     = 0.0; 
    planedBaseStates_.roll[1]  = 0.0; 
    planedBaseStates_.pitch[1] = 0.0; 
    planedBaseStates_.yaw[1]   = 0.0; 

    planedBaseStates_.x[2]     = 0.0; 
    planedBaseStates_.y[2]     = 0.0; 
    planedBaseStates_.z[2]     = 0.0; 
    planedBaseStates_.roll[2]  = 0.0; 
    planedBaseStates_.pitch[2] = 0.0; 
    planedBaseStates_.yaw[2]   = 0.0; 
}

bool SimpleMotion::isWBCMotionFinished() {
    if(timeNowWBC_ <= timeGoalWBC_) {
        return false;
    }
    else {
        return true;
    }
}

void SimpleMotion::WBCMotionRun(LimbsCommand& command, bool& safeGuard) {
    if(!isWBCSetUp) {
        std::cerr << "\n[SimpleMotion::WBCMotionRun]:YOU SHOULD SETUP FIRST!";
        
        for (int i(0); i < 3; i++) {
            command.lf_tau.value[i] = 0;
            command.rf_tau.value[i] = 0;
            command.lh_tau.value[i] = 0;
            command.rh_tau.value[i] = 0;
        }
        return;
    }

    if(!isWBCMotionFinished()) {
        timeNowWBC_ = iteratorWBC_ * timeCycle_;
        CubicSplinePlanForBase();
        if(isWBCSwingSetUp) {
            CubicSplinePlanForSwingFoot();
        }
    }
    else {
        KeepBaseStates();
        if(isWBCSwingSetUp) {
             KeepSwingFootStates();
        }
    }

    iteratorWBC_++;

    desiredDataWBC_.pBody_des << planedBaseStates_.x[0], planedBaseStates_.y[0], planedBaseStates_.z[0];
    desiredDataWBC_.vBody_des << planedBaseStates_.x[1], planedBaseStates_.y[1], planedBaseStates_.z[1];
    desiredDataWBC_.aBody_des << planedBaseStates_.x[2], planedBaseStates_.y[2], planedBaseStates_.z[2];

    desiredDataWBC_.pBody_RPY_des << planedBaseStates_.roll[0], planedBaseStates_.pitch[0], planedBaseStates_.yaw[0];
    desiredDataWBC_.vBody_RPY_des << planedBaseStates_.roll[1], planedBaseStates_.pitch[1], planedBaseStates_.yaw[1];
    desiredDataWBC_.aBody_RPY_des << planedBaseStates_.roll[2], planedBaseStates_.pitch[2], planedBaseStates_.yaw[2];

    for(int i(0); i < 4; i++) {
        desiredDataWBC_.pFoot_des[i] << 0, 0, 0;
        desiredDataWBC_.vFoot_des[i] << 0, 0, 0;
        desiredDataWBC_.aFoot_des[i] << 0, 0, 0;
    }

    if(isWBCSwingSetUp) {
        desiredDataWBC_.pFoot_des[foot_id_] << planedFootStates_.x[0], planedFootStates_.y[0], planedFootStates_.z[0];
        desiredDataWBC_.vFoot_des[foot_id_] << planedFootStates_.x[1], planedFootStates_.y[1], planedFootStates_.z[1];
        desiredDataWBC_.aFoot_des[foot_id_] << planedFootStates_.x[2], planedFootStates_.y[2], planedFootStates_.z[2];
    }
    Eigen::Vector3f qnormal;
    qnormal << -0.04, -0.676, 1.317;
    desiredDataWBC_.pLegJoint_des[legID::LF] = qnormal;
    desiredDataWBC_.pLegJoint_des[legID::LB] = qnormal; 
    desiredDataWBC_.pLegJoint_des[legID::RF] = qnormal;
    desiredDataWBC_.pLegJoint_des[legID::RB] = qnormal;

    desiredDataWBC_.vLegJoint_des[legID::LF].setZero();
    desiredDataWBC_.vLegJoint_des[legID::LB].setZero();
    desiredDataWBC_.vLegJoint_des[legID::RF].setZero();
    desiredDataWBC_.vLegJoint_des[legID::RB].setZero();

    desiredDataWBC_.aLegJoint_des[legID::LF].setZero();
    desiredDataWBC_.aLegJoint_des[legID::LB].setZero();
    desiredDataWBC_.aLegJoint_des[legID::RF].setZero();
    desiredDataWBC_.aLegJoint_des[legID::RB].setZero();
    
    currentStatesWBC_.bodyStateEst.frame_c_rpy_in_world[0] = 0;
    currentStatesWBC_.bodyStateEst.frame_c_rpy_in_world[1] = 0;
    currentStatesWBC_.bodyStateEst.frame_c_rpy_in_world[2] = 0;

    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.w() = 1;
    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.x() = 0;
    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.y() = 0;
    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.z() = 0;

    currentStatesWBC_.bodyStateEst.frame_c_xyz_in_world[0] = 0;
    currentStatesWBC_.bodyStateEst.frame_c_xyz_in_world[1] = 0;
    currentStatesWBC_.bodyStateEst.frame_c_xyz_in_world[2] = 0;

    desiredDataWBC_.contact_force << 0, 0, 135, 
                                     0, 0, 135, 
                                     0, 0, 135, 
                                     0, 0, 135;

    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    wbc_ctrl_->run(&desiredDataWBC_, currentStatesWBC_,tauWBC_);
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cerr << "wbc run time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() << "us\n";

    for (int i(0); i < 3; i++) {
        command.lf_tau.value[i] = tauWBC_[i];
        command.lh_tau.value[i] = tauWBC_[i+3];
        command.rf_tau.value[i] = tauWBC_[i+6];
        command.rh_tau.value[i] = tauWBC_[i+9];
    }

    if(abs(currentStatesWBC_.bodyStateEst.base_pos_world[0] - desiredDataWBC_.pBody_des[0]) > paramf.x_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_pos_world[1] - desiredDataWBC_.pBody_des[1]) > paramf.y_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_pos_world[2] - desiredDataWBC_.pBody_des[2]) > paramf.z_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_rpy_world[0] - desiredDataWBC_.pBody_RPY_des[0]) > paramf.roll_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_rpy_world[1] - desiredDataWBC_.pBody_RPY_des[1]) > paramf.pitch_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_rpy_world[2] - desiredDataWBC_.pBody_RPY_des[2]) > paramf.yaw_delta) {
           
        safeGuard = false;
    }

    // RecordData();
}

void SimpleMotion::WBCSetUpContactForSwingMotion(size_t foot_id) {
    foot_id_ = foot_id;
    currentStatesWBC_.bodyStateEst.contactEstimate << 1, 1, 1, 1;
    desiredDataWBC_.contact_state << 1, 1, 1, 1;

    currentStatesWBC_.bodyStateEst.contactEstimate[foot_id_] = 0;
    desiredDataWBC_.contact_state[foot_id_] = 0;
}

void SimpleMotion::WBCSetUpSwingFootInitialStates(){
    initialBaseStates_.x[0] = currentStatesWBC_.bodyStateEst.base_pos_world[0];
    initialBaseStates_.y[0] = currentStatesWBC_.bodyStateEst.base_pos_world[1];
    initialBaseStates_.z[0] = currentStatesWBC_.bodyStateEst.base_pos_world[2];
    initialBaseStates_.roll[0]  = currentStatesWBC_.bodyStateEst.base_rpy_world[0];
    initialBaseStates_.pitch[0] = currentStatesWBC_.bodyStateEst.base_rpy_world[1];
    initialBaseStates_.yaw[0]   = currentStatesWBC_.bodyStateEst.base_rpy_world[2];

    Q_.head(3) << currentStatesWBC_.bodyStateEst.base_pos_world;
    // Q_.segment(3, 3)  << currentStatesWBC_.bodyStateEst.base_rpy_world;
    // Q_.head(3) << 0, 0, 0;
    // Q_.segment(3, 3)  << 0, 0, 0;
    Q_[3]  = currentStatesWBC_.bodyStateEst.base_orientation_world.x();
    Q_[4]  = currentStatesWBC_.bodyStateEst.base_orientation_world.y();
    Q_[5]  = currentStatesWBC_.bodyStateEst.base_orientation_world.z();
    Q_[18] = currentStatesWBC_.bodyStateEst.base_orientation_world.w();
    
    Q_.segment(6, 3)  << currentStatesWBC_.legStateEst[legID::LF].q;
    Q_.segment(9, 3)  << currentStatesWBC_.legStateEst[legID::LB].q;
    Q_.segment(12, 3) << currentStatesWBC_.legStateEst[legID::RF].q;
    Q_.segment(15, 3) << currentStatesWBC_.legStateEst[legID::RB].q;

    QDot_.head(3) << currentStatesWBC_.bodyStateEst.base_linear_vel_world;
    QDot_.segment(3, 3)  << currentStatesWBC_.bodyStateEst.base_angular_vel_world;
    // QDot_.head(3) << 0, 0, 0;
    // QDot_.segment(3, 3)  << 0, 0, 0;
    QDot_.segment(6, 3)  << currentStatesWBC_.legStateEst[legID::LF].qd;
    QDot_.segment(9, 3)  << currentStatesWBC_.legStateEst[legID::LB].qd;
    QDot_.segment(12, 3) << currentStatesWBC_.legStateEst[legID::RF].qd;
    QDot_.segment(15, 3) << currentStatesWBC_.legStateEst[legID::RB].qd;

    Vec31<float> foot_position = jueying_.swingFootPosition(foot_id_, Q_.cast<double>()).cast<float>();
    initialFootStates_.x[0] = foot_position[0];  
    initialFootStates_.y[0] = foot_position[1];
    initialFootStates_.z[0] = foot_position[2];
}

void SimpleMotion::WBCSetUpSwingFootGoalStatesAll(float base_x, float base_y, float base_z, float base_roll, float base_pitch, float base_yaw,
                                                  float foot_x, float foot_y, float foot_z) {
    goalBaseStates_.x[0] = currentStatesWBC_.bodyStateEst.base_pos_world[0] + base_x;
    goalBaseStates_.y[0] = currentStatesWBC_.bodyStateEst.base_pos_world[1] + base_y;
    goalBaseStates_.z[0] = currentStatesWBC_.bodyStateEst.base_pos_world[2] + base_z;
    goalBaseStates_.roll[0]  = currentStatesWBC_.bodyStateEst.base_rpy_world[0]  + base_roll;
    goalBaseStates_.pitch[0] = currentStatesWBC_.bodyStateEst.base_rpy_world[1]  + base_pitch;
    goalBaseStates_.yaw[0]   = currentStatesWBC_.bodyStateEst.base_rpy_world[2]  + base_yaw;

    goalFootStates_.x[0] = initialFootStates_.x[0] + foot_x;   
    goalFootStates_.y[0] = initialFootStates_.y[0] + foot_y;   
    goalFootStates_.z[0] = initialFootStates_.z[0] + foot_z; 
}

void SimpleMotion::CubicSplinePlanForSwingFoot() {
    planedFootStates_.x = TrajectoryPlan_f(initialFootStates_.x[0], goalFootStates_.x[0], timeGoalWBC_, timeNowWBC_);
    planedFootStates_.y = TrajectoryPlan_f(initialFootStates_.y[0], goalFootStates_.y[0], timeGoalWBC_, timeNowWBC_);
    planedFootStates_.z = TrajectoryPlan_f(initialFootStates_.z[0], goalFootStates_.z[0], timeGoalWBC_, timeNowWBC_);   
}

bool SimpleMotion::WBCSetUpSwingFootMotion(float base_x, float base_y, float base_z, float base_roll, float base_pitch, float base_yaw,
                                           size_t foot_id, float foot_x, float foot_y, float foot_z,
                                           float timeGoal) {
    WBCSetUpContactForSwingMotion(foot_id);
    WBCSetUpSwingFootInitialStates();
    WBCSetUpSwingFootGoalStatesAll(base_x, base_y, base_z, base_roll, base_pitch, base_yaw, foot_x, foot_y, foot_z);
    WBCSetUpGoalTime(timeGoal);
    iteratorWBC_ = 0;
    timeNowWBC_ = 0;

    isWBCSetUp = true;
    isWBCSwingSetUp = true;
    return true; 
}

void SimpleMotion::KeepSwingFootStates() {
    planedFootStates_.x[0] = goalFootStates_.x[0];
    planedFootStates_.y[0] = goalFootStates_.y[0];
    planedFootStates_.z[0] = goalFootStates_.z[0];

    planedFootStates_.x[1] = 0;
    planedFootStates_.y[1] = 0;
    planedFootStates_.z[1] = 0;

    planedFootStates_.x[2] = 0;
    planedFootStates_.y[2] = 0;
    planedFootStates_.z[2] = 0;
}

const std::vector<Vec31<float>> SimpleMotion::RecordData() {
    Eigen::Quaternion<float> quat = currentStatesWBC_.bodyStateEst.base_orientation_world;
    
    
    Q_.head(3) << currentStatesWBC_.bodyStateEst.base_pos_world;
    // Q_.segment(3, 3)  << currentStatesWBC_.bodyStateEst.base_rpy_world;
    Q_.segment(3, 3)  << quat.x(), quat.y(), quat.z();
    // Q_.head(3) << 0, 0, 0;
    // Q_.segment(3, 3)  << 0, 0, 0;
    Q_.segment(6, 3)  << currentStatesWBC_.legStateEst[legID::LF].q;
    Q_.segment(9, 3)  << currentStatesWBC_.legStateEst[legID::LB].q;
    Q_.segment(12, 3) << currentStatesWBC_.legStateEst[legID::RF].q;
    Q_.segment(15, 3) << currentStatesWBC_.legStateEst[legID::RB].q;
    Q_[18] = quat.w();
    feet_result.clear();
    feet_result.reserve(4);
    Vec31<float> foot_position = jueying_.swingFootPosition(legID::LF, Q_.cast<double>()).cast<float>();
    feet_result.emplace_back(foot_position);
    // in_foot_lf_x << desiredDataWBC_.pFoot_des[legID::LF][0] << "\t" << foot_position[0] << "\n";
    // in_foot_lf_y << desiredDataWBC_.pFoot_des[legID::LF][1] << "\t" << foot_position[1] << "\n";
    // in_foot_lf_z << desiredDataWBC_.pFoot_des[legID::LF][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::LF] << "\n";

    foot_position = jueying_.swingFootPosition(legID::LB, Q_.cast<double>()).cast<float>();
    feet_result.emplace_back(foot_position);
    // in_foot_lh_x << desiredDataWBC_.pFoot_des[legID::LB][0] << "\t" << foot_position[0] << "\n";
    // in_foot_lh_y << desiredDataWBC_.pFoot_des[legID::LB][1] << "\t" << foot_position[1] << "\n";
    // in_foot_lh_z << desiredDataWBC_.pFoot_des[legID::LB][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::LB] << "\n";

    foot_position = jueying_.swingFootPosition(legID::RF, Q_.cast<double>()).cast<float>();
    feet_result.emplace_back(foot_position);
    // in_foot_rf_x << desiredDataWBC_.pFoot_des[legID::RF][0] << "\t" << foot_position[0] << "\n";
    // in_foot_rf_y << desiredDataWBC_.pFoot_des[legID::RF][1] << "\t" << foot_position[1] << "\n";
    // in_foot_rf_z << desiredDataWBC_.pFoot_des[legID::RF][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::RF] << "\n";

    foot_position = jueying_.swingFootPosition(legID::RB, Q_.cast<double>()).cast<float>();
    feet_result.emplace_back(foot_position);
    // in_foot_rh_x << desiredDataWBC_.pFoot_des[legID::RB][0] << "\t" << foot_position[0] << "\n";
    // in_foot_rh_y << desiredDataWBC_.pFoot_des[legID::RB][1] << "\t" << foot_position[1] << "\n";
    // in_foot_rh_z << desiredDataWBC_.pFoot_des[legID::RB][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::RB] << "\n";
    // std::cout << "feet_result: " << feet_result.size() << std::endl;
    return feet_result;
    // in_x << desiredDataWBC_.pBody_des[0] << "\t" << currentStatesWBC_.bodyStateEst.base_pos_world[0] << "\n";
    // in_y << desiredDataWBC_.pBody_des[1] << "\t" << currentStatesWBC_.bodyStateEst.base_pos_world[1] << "\n";
    // in_z << desiredDataWBC_.pBody_des[2] << "\t" << currentStatesWBC_.bodyStateEst.base_pos_world[2] << "\n";
    // in_roll  << desiredDataWBC_.pBody_RPY_des[0] << "\t" << currentStatesWBC_.bodyStateEst.base_rpy_world[0] << "\n";
    // in_pitch << desiredDataWBC_.pBody_RPY_des[1] << "\t" << currentStatesWBC_.bodyStateEst.base_rpy_world[1] << "\n";
    // in_yaw   << desiredDataWBC_.pBody_RPY_des[2] << "\t" << currentStatesWBC_.bodyStateEst.base_rpy_world[2] << "\n";

    // in_x_vel << desiredDataWBC_.vBody_des[0] << "\t" << currentStatesWBC_.bodyStateEst.base_linear_vel_world[0] << "\n";
    // in_y_vel << desiredDataWBC_.vBody_des[1] << "\t" << currentStatesWBC_.bodyStateEst.base_linear_vel_world[1] << "\n";
    // in_z_vel << desiredDataWBC_.vBody_des[2] << "\t" << currentStatesWBC_.bodyStateEst.base_linear_vel_world[2] << "\n";
    // in_roll_vel  << desiredDataWBC_.vBody_RPY_des[0] << "\t" << currentStatesWBC_.bodyStateEst.base_angular_vel_world[0] << "\n";
    // in_pitch_vel << desiredDataWBC_.vBody_RPY_des[1] << "\t" << currentStatesWBC_.bodyStateEst.base_angular_vel_world[1] << "\n";
    // in_yaw_vel   << desiredDataWBC_.vBody_RPY_des[2] << "\t" << currentStatesWBC_.bodyStateEst.base_angular_vel_world[2] << "\n";

    // Q_.head(3) << currentStatesWBC_.bodyStateEst.base_pos_world;
    // Q_.segment(3, 3)  << currentStatesWBC_.bodyStateEst.base_rpy_world;
    // Q_.segment(6, 3)  << currentStatesWBC_.legStateEst[legID::LF].q;
    // Q_.segment(9, 3)  << currentStatesWBC_.legStateEst[legID::LB].q;
    // Q_.segment(12, 3) << currentStatesWBC_.legStateEst[legID::RF].q;
    // Q_.segment(15, 3) << currentStatesWBC_.legStateEst[legID::RB].q;

    // Vec31<float> foot_position = jueying_.swingFootPosition(legID::LF, Q_.cast<double>()).cast<float>();
    // in_foot_lf_x << desiredDataWBC_.pFoot_des[legID::LF][0] << "\t" << foot_position[0] << "\n";
    // in_foot_lf_y << desiredDataWBC_.pFoot_des[legID::LF][1] << "\t" << foot_position[1] << "\n";
    // in_foot_lf_z << desiredDataWBC_.pFoot_des[legID::LF][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::LF] << "\n";

    // foot_position = jueying_.swingFootPosition(legID::RF, Q_.cast<double>()).cast<float>();
    // in_foot_rf_x << desiredDataWBC_.pFoot_des[legID::RF][0] << "\t" << foot_position[0] << "\n";
    // in_foot_rf_y << desiredDataWBC_.pFoot_des[legID::RF][1] << "\t" << foot_position[1] << "\n";
    // in_foot_rf_z << desiredDataWBC_.pFoot_des[legID::RF][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::RF] << "\n";

    // foot_position = jueying_.swingFootPosition(legID::LB, Q_.cast<double>()).cast<float>();
    // in_foot_lh_x << desiredDataWBC_.pFoot_des[legID::LB][0] << "\t" << foot_position[0] << "\n";
    // in_foot_lh_y << desiredDataWBC_.pFoot_des[legID::LB][1] << "\t" << foot_position[1] << "\n";
    // in_foot_lh_z << desiredDataWBC_.pFoot_des[legID::LB][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::LB] << "\n";

    // foot_position = jueying_.swingFootPosition(legID::RB, Q_.cast<double>()).cast<float>();
    // in_foot_rh_x << desiredDataWBC_.pFoot_des[legID::RB][0] << "\t" << foot_position[0] << "\n";
    // in_foot_rh_y << desiredDataWBC_.pFoot_des[legID::RB][1] << "\t" << foot_position[1] << "\n";
    // in_foot_rh_z << desiredDataWBC_.pFoot_des[legID::RB][2] << "\t" << foot_position[2] << "\t" << currentStatesWBC_.bodyStateEst.contactEstimate[legID::RB] << "\n";
}

void SimpleMotion::UpdateMPCMsg(float time_stamp) {
    assert(mpcMsgPtr_ != nullptr);
    indexMPCStateTime_ = 0;
    indexMPCSwitchTime_ = 0;
    timeUpdateMPC_ = time_stamp;

}

void SimpleMotion::UpdateControlFrame(const EstimatorOutput& input) {
    currentStatesWBC_.bodyStateEst.frame_c_rpy_in_world[0] = input.frame_c_rpy_in_world[0];
    currentStatesWBC_.bodyStateEst.frame_c_rpy_in_world[1] = input.frame_c_rpy_in_world[1];
    currentStatesWBC_.bodyStateEst.frame_c_rpy_in_world[2] = input.frame_c_rpy_in_world[2];

    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.w() = input.frame_c_quat_in_world.w();
    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.x() = input.frame_c_quat_in_world.x();
    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.y() = input.frame_c_quat_in_world.y();
    currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.z() = input.frame_c_quat_in_world.z();

    currentStatesWBC_.bodyStateEst.frame_c_xyz_in_world[0] = input.frame_c_xyz_in_world[0];
    currentStatesWBC_.bodyStateEst.frame_c_xyz_in_world[1] = input.frame_c_xyz_in_world[1];
    currentStatesWBC_.bodyStateEst.frame_c_xyz_in_world[2] = input.frame_c_xyz_in_world[2];
}

size_t SimpleMotion::MPCWBCRun(float time_stamp, LimbsCommand& command, bool& safeGuard) {

    // std::cerr << "6" << "\n";
    // StateTime
    indexMPCStateTime_ = 0;
    while ((time_stamp) > mpcMsgPtr_->stateTime[indexMPCStateTime_] && indexMPCStateTime_ < mpcMsgPtr_->stateTime.size() - 1) {
        indexMPCStateTime_++;
    }
    // std::cout << "indexMPCStateTime_ = " << indexMPCStateTime_ << "\n";
    // std::cerr << "7" << "\n";
    // Contact
    if(indexMPCStateTime_ >= mpcMsgPtr_->stateTime.size()) {
        indexMPCStateTime_ = mpcMsgPtr_->stateTime.size() - 1;
        std::cerr << "\n[yjy: SimpleMotion::MPCWBCRun] MPC IS OUT OF TIME!\n";
    }
    // std::cerr << "8" << "\n";
    if(time_stamp < mpcMsgPtr_->switchTime[0]) {
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::LF] = mpcMsgPtr_->firstGait[0];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::RF] = mpcMsgPtr_->firstGait[1];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::LB] = mpcMsgPtr_->firstGait[2];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::RB] = mpcMsgPtr_->firstGait[3];
        desiredDataWBC_.contact_state[legID::LF] = mpcMsgPtr_->firstGait[0];
        desiredDataWBC_.contact_state[legID::RF] = mpcMsgPtr_->firstGait[1];
        desiredDataWBC_.contact_state[legID::LB] = mpcMsgPtr_->firstGait[2];
        desiredDataWBC_.contact_state[legID::RB] = mpcMsgPtr_->firstGait[3];
    }
    else if(time_stamp < mpcMsgPtr_->switchTime[1] && time_stamp >= mpcMsgPtr_->switchTime[0]) {
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::LF] = mpcMsgPtr_->secondGait[0];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::RF] = mpcMsgPtr_->secondGait[1];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::LB] = mpcMsgPtr_->secondGait[2];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::RB] = mpcMsgPtr_->secondGait[3];
        desiredDataWBC_.contact_state[legID::LF] = mpcMsgPtr_->secondGait[0];
        desiredDataWBC_.contact_state[legID::RF] = mpcMsgPtr_->secondGait[1];
        desiredDataWBC_.contact_state[legID::LB] = mpcMsgPtr_->secondGait[2];
        desiredDataWBC_.contact_state[legID::RB] = mpcMsgPtr_->secondGait[3];
    }
    else if(time_stamp >= mpcMsgPtr_->switchTime[1]){
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::LF] = mpcMsgPtr_->thirdGait[0];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::RF] = mpcMsgPtr_->thirdGait[1];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::LB] = mpcMsgPtr_->thirdGait[2];
        currentStatesWBC_.bodyStateEst.contactEstimate[legID::RB] = mpcMsgPtr_->thirdGait[3];
        desiredDataWBC_.contact_state[legID::LF] = mpcMsgPtr_->thirdGait[0];
        desiredDataWBC_.contact_state[legID::RF] = mpcMsgPtr_->thirdGait[1];
        desiredDataWBC_.contact_state[legID::LB] = mpcMsgPtr_->thirdGait[2];
        desiredDataWBC_.contact_state[legID::RB] = mpcMsgPtr_->thirdGait[3];  
    }
    else {
        currentStatesWBC_.bodyStateEst.contactEstimate << 1, 1, 1, 1;
        desiredDataWBC_.contact_state << 1, 1, 1, 1;
        std::cerr << "\n[yjy: SimpleMotion::MPCWBCRun] MPC IS OUT OF SWITCH TIME!\n";
    }
    // std::cerr << "9" << "\n";

    // Desired WBC Trajectories
    desiredDataWBC_.pBody_des = mpcMsgPtr_->basePosition[indexMPCStateTime_].head(3);
    desiredDataWBC_.vBody_des = mpcMsgPtr_->baseVelocity[indexMPCStateTime_].head(3);
    desiredDataWBC_.aBody_des = mpcMsgPtr_->baseAcceleration[indexMPCStateTime_].head(3);

    desiredDataWBC_.pBody_RPY_des = mpcMsgPtr_->basePosition[indexMPCStateTime_].tail(3);
    desiredDataWBC_.vBody_RPY_des = mpcMsgPtr_->baseVelocity[indexMPCStateTime_].tail(3);
    desiredDataWBC_.aBody_RPY_des = mpcMsgPtr_->baseAcceleration[indexMPCStateTime_].tail(3);

    // std::cout << "pBody_RPY_des " << desiredDataWBC_.pBody_RPY_des.transpose() << "\n";
    // std::cout << "vBody_RPY_des " << desiredDataWBC_.vBody_RPY_des.transpose() << "\n";

    desiredDataWBC_.pFoot_des[legID::LF] = mpcMsgPtr_->swingFeetPosition[indexMPCStateTime_][0];
    desiredDataWBC_.pFoot_des[legID::RF] = mpcMsgPtr_->swingFeetPosition[indexMPCStateTime_][1];
    desiredDataWBC_.pFoot_des[legID::LB] = mpcMsgPtr_->swingFeetPosition[indexMPCStateTime_][2];
    desiredDataWBC_.pFoot_des[legID::RB] = mpcMsgPtr_->swingFeetPosition[indexMPCStateTime_][3];

    desiredDataWBC_.vFoot_des[legID::LF] = mpcMsgPtr_->swingFeetVelocity[indexMPCStateTime_][0];
    desiredDataWBC_.vFoot_des[legID::RF] = mpcMsgPtr_->swingFeetVelocity[indexMPCStateTime_][1];
    desiredDataWBC_.vFoot_des[legID::LB] = mpcMsgPtr_->swingFeetVelocity[indexMPCStateTime_][2];
    desiredDataWBC_.vFoot_des[legID::RB] = mpcMsgPtr_->swingFeetVelocity[indexMPCStateTime_][3];

    desiredDataWBC_.aFoot_des[legID::LF] = mpcMsgPtr_->swingFeetAcceleration[indexMPCStateTime_][0];
    desiredDataWBC_.aFoot_des[legID::RF] = mpcMsgPtr_->swingFeetAcceleration[indexMPCStateTime_][1];
    desiredDataWBC_.aFoot_des[legID::LB] = mpcMsgPtr_->swingFeetAcceleration[indexMPCStateTime_][2];
    desiredDataWBC_.aFoot_des[legID::RB] = mpcMsgPtr_->swingFeetAcceleration[indexMPCStateTime_][3];

    desiredDataWBC_.pLegJoint_des[legID::LF] = mpcMsgPtr_->actJointPos[indexMPCStateTime_].head(3);
    desiredDataWBC_.pLegJoint_des[legID::LB] = mpcMsgPtr_->actJointPos[indexMPCStateTime_].segment(3, 3);
    desiredDataWBC_.pLegJoint_des[legID::RF] = mpcMsgPtr_->actJointPos[indexMPCStateTime_].segment(6, 3);
    desiredDataWBC_.pLegJoint_des[legID::RB] = mpcMsgPtr_->actJointPos[indexMPCStateTime_].tail(3);

    std::cout << "des LH:" << desiredDataWBC_.pLegJoint_des[legID::LB].transpose() << "\n";
    // std::cout << "cur LH:" << currentStatesWBC_.legStateEst[legID::LB].q.transpose() << "\n";

    desiredDataWBC_.vLegJoint_des[legID::LF] = mpcMsgPtr_->actJointVel[indexMPCStateTime_].head(3);
    desiredDataWBC_.vLegJoint_des[legID::LB] = mpcMsgPtr_->actJointVel[indexMPCStateTime_].segment(3, 3);
    desiredDataWBC_.vLegJoint_des[legID::RF] = mpcMsgPtr_->actJointVel[indexMPCStateTime_].segment(6, 3);
    desiredDataWBC_.vLegJoint_des[legID::RB] = mpcMsgPtr_->actJointVel[indexMPCStateTime_].tail(3);

    // std::cout << "mpcMsgPtr_->actJointVel " << mpcMsgPtr_->actJointVel[indexMPCStateTime_].transpose() << "\n";

    // desiredDataWBC_.aLegJoint_des[legID::LF] = mpcMsgPtr_->actJointAcc[indexMPCStateTime_].head(3);
    // desiredDataWBC_.aLegJoint_des[legID::LB] = mpcMsgPtr_->actJointAcc[indexMPCStateTime_].segment(3, 3);
    // desiredDataWBC_.aLegJoint_des[legID::RF] = mpcMsgPtr_->actJointAcc[indexMPCStateTime_].segment(6, 3);
    // desiredDataWBC_.aLegJoint_des[legID::RB] = mpcMsgPtr_->actJointAcc[indexMPCStateTime_].tail(3);

    desiredDataWBC_.contact_force = mpcMsgPtr_->inputForce;

    std::cerr << "pBody_RPY_des:" << desiredDataWBC_.pBody_RPY_des[0] << "\t" << desiredDataWBC_.pBody_RPY_des[1] <<"\n";

    // desiredDataWBC_.pBody_RPY_des[0] = slope_delta_roll;
    // desiredDataWBC_.pBody_RPY_des[1] = slope_delta_pitch;


    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    wbc_ctrl_->run(&desiredDataWBC_, currentStatesWBC_,tauWBC_);
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cerr << "wbc run time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() << "us\n";

    command.lf_tau.value[0] = tauWBC_[0] + paramf.Kp_joint_lf[0] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][0] - currentStatesWBC_.legStateEst[legID::LF].q[0]) + paramf.Kd_joint_lf[0] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][0] - currentStatesWBC_.legStateEst[legID::LF].qd[0]);
    command.lf_tau.value[1] = tauWBC_[1] + paramf.Kp_joint_lf[1] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][1] - currentStatesWBC_.legStateEst[legID::LF].q[1]) + paramf.Kd_joint_lf[1] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][1] - currentStatesWBC_.legStateEst[legID::LF].qd[1]);
    command.lf_tau.value[2] = tauWBC_[2] + paramf.Kp_joint_lf[2] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][2] - currentStatesWBC_.legStateEst[legID::LF].q[2]) + paramf.Kd_joint_lf[2] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][2] - currentStatesWBC_.legStateEst[legID::LF].qd[2]);

    command.lh_tau.value[0] = tauWBC_[3] + paramf.Kp_joint_lh[0] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][3] - currentStatesWBC_.legStateEst[legID::LB].q[0]) + paramf.Kd_joint_lh[0] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][3] - currentStatesWBC_.legStateEst[legID::LB].qd[0]);
    command.lh_tau.value[1] = tauWBC_[4] + paramf.Kp_joint_lh[1] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][4] - currentStatesWBC_.legStateEst[legID::LB].q[1]) + paramf.Kd_joint_lh[1] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][4] - currentStatesWBC_.legStateEst[legID::LB].qd[1]);
    command.lh_tau.value[2] = tauWBC_[5] + paramf.Kp_joint_lh[2] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][5] - currentStatesWBC_.legStateEst[legID::LB].q[2]) + paramf.Kd_joint_lh[2] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][5] - currentStatesWBC_.legStateEst[legID::LB].qd[2]);

    command.rf_tau.value[0] = tauWBC_[6] + paramf.Kp_joint_rf[0] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][6] - currentStatesWBC_.legStateEst[legID::RF].q[0]) + paramf.Kd_joint_rf[0] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][6] - currentStatesWBC_.legStateEst[legID::RF].qd[0]);
    command.rf_tau.value[1] = tauWBC_[7] + paramf.Kp_joint_rf[1] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][7] - currentStatesWBC_.legStateEst[legID::RF].q[1]) + paramf.Kd_joint_rf[1] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][7] - currentStatesWBC_.legStateEst[legID::RF].qd[1]);
    command.rf_tau.value[2] = tauWBC_[8] + paramf.Kp_joint_rf[2] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][8] - currentStatesWBC_.legStateEst[legID::RF].q[2]) + paramf.Kd_joint_rf[2] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][8] - currentStatesWBC_.legStateEst[legID::RF].qd[2]);

    command.rh_tau.value[0] = tauWBC_[9]  + paramf.Kp_joint_rh[0] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][9]  - currentStatesWBC_.legStateEst[legID::RB].q[0]) + paramf.Kd_joint_rh[0] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][9]  - currentStatesWBC_.legStateEst[legID::RB].qd[0]);
    command.rh_tau.value[1] = tauWBC_[10] + paramf.Kp_joint_rh[1] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][10] - currentStatesWBC_.legStateEst[legID::RB].q[1]) + paramf.Kd_joint_rh[1] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][10] - currentStatesWBC_.legStateEst[legID::RB].qd[1]);
    command.rh_tau.value[2] = tauWBC_[11] + paramf.Kp_joint_rh[2] * (mpcMsgPtr_->actJointPos[indexMPCStateTime_][11] - currentStatesWBC_.legStateEst[legID::RB].q[2]) + paramf.Kd_joint_rh[2] * (mpcMsgPtr_->actJointVel[indexMPCStateTime_][11] - currentStatesWBC_.legStateEst[legID::RB].qd[2]);
    
    if(abs(currentStatesWBC_.bodyStateEst.base_pos_world[0] - desiredDataWBC_.pBody_des[0]) > paramf.x_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_pos_world[1] - desiredDataWBC_.pBody_des[1]) > paramf.y_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_pos_world[2] - desiredDataWBC_.pBody_des[2]) > paramf.z_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_rpy_world[0] - desiredDataWBC_.pBody_RPY_des[0]) > paramf.roll_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_rpy_world[1] - desiredDataWBC_.pBody_RPY_des[1]) > paramf.pitch_delta ||
       abs(currentStatesWBC_.bodyStateEst.base_rpy_world[2] - desiredDataWBC_.pBody_RPY_des[2]) > paramf.yaw_delta) {
           
        safeGuard = false;
    }
    RecordData();
    return indexMPCStateTime_;
}

void SimpleMotion::PDSafeGuardSetUpMotion() {
    for (int i(0); i < 3; i++) {
        planedLimbsStates_.lf_pos.value[i] = currentLimbsStates_.lf_pos.value[i];
        planedLimbsStates_.rf_pos.value[i] = currentLimbsStates_.rf_pos.value[i];
        planedLimbsStates_.lh_pos.value[i] = currentLimbsStates_.lh_pos.value[i];
        planedLimbsStates_.rh_pos.value[i] = currentLimbsStates_.rh_pos.value[i];

        planedLimbsStates_.lf_vel.value[i] = 0.0;
        planedLimbsStates_.rf_vel.value[i] = 0.0;
        planedLimbsStates_.lh_vel.value[i] = 0.0;
        planedLimbsStates_.rh_vel.value[i] = 0.0;
    } 
}

void SimpleMotion::PDSafeGuardRun(LimbsCommand& command) {
    command.lf_tau.value[0] = paramf.Kp_lf_haa_pd_safe * (planedLimbsStates_.lf_pos.value[0] - currentLimbsStates_.lf_pos.value[0]) + paramf.Kd_lf_haa_pd_safe * (planedLimbsStates_.lf_vel.value[0] - currentLimbsStates_.lf_vel.value[0]); // LF LH RF RH
    command.lf_tau.value[1] = paramf.Kp_lf_hfe_pd_safe * (planedLimbsStates_.lf_pos.value[1] - currentLimbsStates_.lf_pos.value[1]) + paramf.Kd_lf_hfe_pd_safe * (planedLimbsStates_.lf_vel.value[1] - currentLimbsStates_.lf_vel.value[1]); 
    command.lf_tau.value[2] = paramf.Kp_lf_kfe_pd_safe * (planedLimbsStates_.lf_pos.value[2] - currentLimbsStates_.lf_pos.value[2]) + paramf.Kd_lf_kfe_pd_safe * (planedLimbsStates_.lf_vel.value[2] - currentLimbsStates_.lf_vel.value[2]); 

    command.rf_tau.value[0] = paramf.Kp_rf_haa_pd_safe * (planedLimbsStates_.rf_pos.value[0] - currentLimbsStates_.rf_pos.value[0]) + paramf.Kd_rf_haa_pd_safe * (planedLimbsStates_.rf_vel.value[0] - currentLimbsStates_.rf_vel.value[0]); // LF LH RF RH
    command.rf_tau.value[1] = paramf.Kp_rf_hfe_pd_safe * (planedLimbsStates_.rf_pos.value[1] - currentLimbsStates_.rf_pos.value[1]) + paramf.Kd_rf_hfe_pd_safe * (planedLimbsStates_.rf_vel.value[1] - currentLimbsStates_.rf_vel.value[1]); 
    command.rf_tau.value[2] = paramf.Kp_rf_kfe_pd_safe * (planedLimbsStates_.rf_pos.value[2] - currentLimbsStates_.rf_pos.value[2]) + paramf.Kd_rf_kfe_pd_safe * (planedLimbsStates_.rf_vel.value[2] - currentLimbsStates_.rf_vel.value[2]); 

    command.lh_tau.value[0] = paramf.Kp_lh_haa_pd_safe * (planedLimbsStates_.lh_pos.value[0] - currentLimbsStates_.lh_pos.value[0]) + paramf.Kd_lh_haa_pd_safe * (planedLimbsStates_.lh_vel.value[0] - currentLimbsStates_.lh_vel.value[0]); // LF LH RF RH
    command.lh_tau.value[1] = paramf.Kp_lh_hfe_pd_safe * (planedLimbsStates_.lh_pos.value[1] - currentLimbsStates_.lh_pos.value[1]) + paramf.Kd_lh_hfe_pd_safe * (planedLimbsStates_.lh_vel.value[1] - currentLimbsStates_.lh_vel.value[1]); 
    command.lh_tau.value[2] = paramf.Kp_lh_kfe_pd_safe * (planedLimbsStates_.lh_pos.value[2] - currentLimbsStates_.lh_pos.value[2]) + paramf.Kd_lh_kfe_pd_safe * (planedLimbsStates_.lh_vel.value[2] - currentLimbsStates_.lh_vel.value[2]); 

    command.rh_tau.value[0] = paramf.Kp_rh_haa_pd_safe * (planedLimbsStates_.rh_pos.value[0] - currentLimbsStates_.rh_pos.value[0]) + paramf.Kd_rh_haa_pd_safe * (planedLimbsStates_.rh_vel.value[0] - currentLimbsStates_.rh_vel.value[0]); // LF LH RF RH
    command.rh_tau.value[1] = paramf.Kp_rh_hfe_pd_safe * (planedLimbsStates_.rh_pos.value[1] - currentLimbsStates_.rh_pos.value[1]) + paramf.Kd_rh_hfe_pd_safe * (planedLimbsStates_.rh_vel.value[1] - currentLimbsStates_.rh_vel.value[1]); 
    command.rh_tau.value[2] = paramf.Kp_rh_kfe_pd_safe * (planedLimbsStates_.rh_pos.value[2] - currentLimbsStates_.rh_pos.value[2]) + paramf.Kd_rh_kfe_pd_safe * (planedLimbsStates_.rh_vel.value[2] - currentLimbsStates_.rh_vel.value[2]); 
}

TerrainEstData SimpleMotion::TerrainEst(const Vec41<int>& contact_flag){
    Eigen::Matrix<float,19,1> q;
    Eigen::Quaternion<float> quat = currentStatesWBC_.bodyStateEst.base_orientation_world;
    // q.head(3) << 0,0,0;
    q.head(3) = currentStatesWBC_.bodyStateEst.base_pos_world;
    q.segment(3, 3)  << quat.x(), quat.y(), quat.z();
    q.segment(6, 3)  << currentStatesWBC_.legStateEst[legID::LF].q;
    q.segment(9, 3)  << currentStatesWBC_.legStateEst[legID::LB].q;
    q.segment(12, 3) << currentStatesWBC_.legStateEst[legID::RF].q;
    q.segment(15, 3) << currentStatesWBC_.legStateEst[legID::RB].q;
    q[18] = quat.w();

    DVec<float> foot_lf, foot_lh, foot_rf, foot_rh;
    foot_lf = jueying_.swingFootPosition(legID::LF, q.cast<double>()).cast<float>();
    foot_lh = jueying_.swingFootPosition(legID::LB, q.cast<double>()).cast<float>();
    foot_rf = jueying_.swingFootPosition(legID::RF, q.cast<double>()).cast<float>();
    foot_rh = jueying_.swingFootPosition(legID::RB, q.cast<double>()).cast<float>();

    // cout << "foot_lf: " << foot_lf.transpose() << endl;

    Vec31<float> terrParam = terrEst.run(foot_lf, foot_lh, foot_rf, foot_rh, contact_flag);
    Vec31<float> terrNormal;
    terrNormal << terrParam[0], terrParam[1], 1.f;
    terrNormal.normalize();
    // cout << "base_pos_world: " << q.head(3).transpose() << endl;
    // cout << "terrEst:" << terrNormal.transpose() << "\tterrainParams: " << terrParam.transpose() << "\n";

    Vec31<float> xNormal = currentStatesWBC_.bodyStateEst.base_orientation_world.toRotationMatrix().col(0);
    // std::cout << "xNormal: " << xNormal.transpose() << std::endl;
    Vec31<float> xAxis = xNormal - xNormal.dot(terrNormal) * terrNormal;
    xAxis.normalize();
    Vec31<float> yAxis = terrNormal.cross(xAxis);
    yAxis.normalize();terrNormal.normalize();
    Mat3<float> terrFramToWorld;xAxis.normalize();
    terrFramToWorld << xAxis, yAxis, terrNormal;
    Eigen::Quaternionf terrainQuat(terrFramToWorld);

    TerrainEstData terrainEstData;
    terrainEstData.terrainQuat = terrainQuat;
    terrainEstData.terrainParams = terrParam;
    terrainEstData.feetHeight = terrEst.getFeetHeight(); //lf lh rf rh 

    currentStatesWBC_.bodyStateEst.terrain_orientation = terrainQuat;

    return terrainEstData;


    // std::cout << "terrFramToWorld**\n " << terrFramToWorld*terrFramToWorld.transpose() << std::endl;
    // std::cout << "terrFramToWorld det: " << terrFramToWorld.determinant() << std::endl;

    // float pitch = atan2(terrNormal[0], 1);
    // float roll = atan2(terrNormal[1], 1);
    // std::cout << "terrain est: " << pitch << " " << roll << std::endl;
    

    // Mat3<float> terrFramToWorld;
    // float a = terrNormal[0], b = terrNormal[1];
    // float coff_a = std::sqrt(1+a*a),
    //       coff_b = std::sqrt(1+b*b),
    //       coff_c = std::sqrt(1+a*a+b*b);

    // terrFramToWorld << 1/coff_a, 0, a/coff_a,
    //                     0, 1/coff_b, b/coff_b,
    //                     -a/coff_c, -b/coff_c, 1/coff_c;
    
    // Vec31<float> rpy = terrFramToWorld.eulerAngles(0,1,2);
    // Eigen::Quaternion<float> quat_terr(currentStatesWBC_.bodyStateEst.frame_c_quat_in_world.toRotationMatrix() 
    //                                         * terrFramToWorld);
    // Vec31<float> rpy_terr = quaternionTOrpy(quat_terr);

    // std::cout << "r: " << rpy[0] << " p: " << rpy[1] << " y: " << rpy[2] <<
    //               "\t p: " << slope_delta_pitch << "  rpy:---\n" << rpy_terr <<"\n";  

    
    // slope_delta_roll  = std::atan2(terrNormal[1],1);
    // slope_delta_pitch = std::atan2(terrNormal[0], std::sqrt(1 + terrNormal[1] * terrNormal[1]));

    // std::cout << "theta_x: " << slope_delta_roll
    //               << "\ttheta_y: " << slope_delta_pitch << "\n";  

    // cout << "rpy_real:\n" << rpy_in_world << "\n";
    // terr << terrNormal[0] << "\t" << terrNormal[1] << "\n";  
}