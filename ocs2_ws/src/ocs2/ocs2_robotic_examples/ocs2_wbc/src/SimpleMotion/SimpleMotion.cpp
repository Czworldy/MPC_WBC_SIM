#include "ocs2_wbc/SimpleMotion/SimpleMotion.h"

namespace ocs2 {
namespace wbc {

SimpleMotion::SimpleMotion(const UserParameter& userParameter, bool verbose) : 
  paramf(userParameter),
  verbose_(verbose)
  { timeCycle_ = paramf.cycle_time; }

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

bool SimpleMotion::PDSetUpMotion(scalar_t angle_haa, scalar_t angle_hfe, scalar_t angle_kfe, scalar_t timeGoal) {
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

void SimpleMotion::PDSetUpGoalLimbsAngles(scalar_t angle_haa, scalar_t angle_hfe, scalar_t angle_kfe) {
    goalLimbsAngles_.lf_pos.value[0] = angle_haa; goalLimbsAngles_.lf_pos.value[1] = angle_hfe; goalLimbsAngles_.lf_pos.value[2] = angle_kfe;
    goalLimbsAngles_.rf_pos.value[0] = angle_haa; goalLimbsAngles_.rf_pos.value[1] = angle_hfe; goalLimbsAngles_.rf_pos.value[2] = angle_kfe;
    goalLimbsAngles_.lh_pos.value[0] = angle_haa; goalLimbsAngles_.lh_pos.value[1] = angle_hfe; goalLimbsAngles_.lh_pos.value[2] = angle_kfe;
    goalLimbsAngles_.rh_pos.value[0] = angle_haa; goalLimbsAngles_.rh_pos.value[1] = angle_hfe; goalLimbsAngles_.rh_pos.value[2] = angle_kfe;
}

void SimpleMotion::CubicSplinePlanForLimbs() {
    vector3_t lf_Traj[3];
    vector3_t rf_Traj[3];
    vector3_t lh_Traj[3];
    vector3_t rh_Traj[3]; 

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

vector3_t SimpleMotion::TrajectoryPlan_d(scalar_t startPoint, scalar_t finalPoint, scalar_t finalTime, scalar_t time_traj){
  scalar_t  a_0, a_1, a_2, a_3;
  vector3_t point_inter;
  a_0 = startPoint;
	a_1 = 0;
	a_2 = 3* (finalPoint - startPoint)/(pow(finalTime,2));
	a_3 = -2* (finalPoint - startPoint)/(powf(finalTime,3));

	point_inter[0] = a_0 + a_1 * time_traj + a_2 *pow(time_traj,2) + a_3 * pow(time_traj,3);//p
	point_inter[1] = a_1 + a_2 * 2 * time_traj + a_3 * 3 * pow(time_traj,2);//v
	point_inter[2] = a_2 * 2 + a_3 * 3 * 2 * time_traj;//a

  return point_inter;
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

//contact_flag lf lh rf rh 
//eePos:{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}; 
TerrainEstData SimpleMotion::TerrainEst(const Eigen::Vector4i& contact_flag, const std::vector<vector3_t>& eePos, 
                                        const Eigen::Quaternion<scalar_t>& baseOriWorld) {


    // cout << "foot_lf: " << foot_lf.transpose() << endl;
    //contact_flag lf lh rf rh 
    // vector3_t terrParam = terrEst.run(foot_lf, foot_lh, foot_rf, foot_rh, contact_flag);
    vector3_t terrParam = terrEst.run(eePos[0], eePos[2], eePos[1], eePos[3], contact_flag);
    vector3_t terrNormal;
    terrNormal << terrParam[0], terrParam[1], 1.f;
    terrNormal.normalize();
    // cout << "base_pos_world: " << q.head(3).transpose() << endl;
    // cout << "terrEst:" << terrNormal.transpose() << "\tterrainParams: " << terrParam.transpose() << "\n";

    vector3_t xNormal = baseOriWorld.toRotationMatrix().col(0);
    // std::cout << "xNormal: " << xNormal.transpose() << std::endl;
    vector3_t xAxis = xNormal - xNormal.dot(terrNormal) * terrNormal;
    xAxis.normalize();
    vector3_t yAxis = terrNormal.cross(xAxis);
    yAxis.normalize();terrNormal.normalize();
    matrix3_t terrFramToWorld;xAxis.normalize();
    terrFramToWorld << xAxis, yAxis, terrNormal;
    Eigen::Quaternionf terrainQuat(terrFramToWorld.cast<float>());

    TerrainEstData terrainEstData;
    terrainEstData.terrainQuat = terrainQuat;
    terrainEstData.terrainParams = terrParam.cast<float>();
    terrainEstData.feetHeight = terrEst.getFeetHeight().cast<float>(); //lf lh rf rh 

    //contact_flag lf lh rf rh 
    //eePos:{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}; 
    terrainEstData.stanceLegs[0] = (bool) contact_flag[0];
    terrainEstData.stanceLegs[1] = (bool) contact_flag[2];
    terrainEstData.stanceLegs[2] = (bool) contact_flag[1];
    terrainEstData.stanceLegs[3] = (bool) contact_flag[3];

    return terrainEstData;

 
}


}
}