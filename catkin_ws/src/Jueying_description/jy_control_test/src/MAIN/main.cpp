// WBC
#include "simpleMotion.h"
// C++
#include <iostream>
#include <unistd.h>
#include <memory>
#include <chrono>
// ROS
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
// #include "gazebo_msgs/LinkStates.h"   //yjy::可以不用LinkStates而使用ModelStates
#include "gazebo_msgs/ModelStates.h"
#include "ocs2_msgs/mpc_wbc_conversion.h"
#include <gazebo_msgs/ContactsState.h>

// Global Variables
conversionData mpcData;
EstimatorOutput estStatesOutput;
float timeStampCur;
Eigen::Matrix<double, 3, 1> basePosWorldCur;
Eigen::Matrix<double, 3, 1> baseLinearVelWorldCur;
Eigen::Matrix<double, 3, 1> baseLinearVelBodyCur;
Eigen::Quaterniond baseOriWorldCur;
Eigen::Matrix<double, 3, 1> baseAngularVelWorldCur;
Eigen::Matrix<double, 3, 1> baseAngularVelBodyCur;
LimbsContacts contactsCur;
LimbsPosVel jointStatesCur;
Eigen::Matrix<double, 3, 3> terrainRotMat;
std::shared_ptr<SimpleMotion> simpleMotion;
LimbsCommand command;
const int cycle_freq = 400;
bool verbose(false);
bool isMPC(false);
bool isMPCMsgUpdate(false);
bool isJointMsg(false);
bool isGazeboMsg(false);

// State Mechine for PD Control
enum ControlFlag {
    kWaitForMsg = 0,
    kPDWaitForStanding = 1,
    kPDStandUpMotion = 2,
    kWBCBaseMotion = 3,
	kWBCMPC = 4,
	kSafeState = 5,
};
int robotState = 0;
const double timePDWaitForStanding(1.0);
const double timePDStandUpMotion(1.0);
const double timeWBCBaseMotion(4.0);

const double haa_PDWaitForStanding(0);
const double hfe_PDWaitForStanding(-1.23);
const double kfe_PDWaitForStanding(2.79);

const double haa_PDStandUpMotion(0);
const double hfe_PDStandUpMotion(-0.95);
const double kfe_PDStandUpMotion(1.7);

const double xBase(0.0);
const double yBase(-0.05);
const double zBase(0.0);
const double rollBase(0.);
const double pitchBase(0.);
const double yawBase(0.);

bool isSetUp_PDWaitForStanding(false);
bool isSetUp_PDStandUpMotion(false);
bool isSetUp_SafeState(false);
bool isStandUp(false);
bool isSetUp_WBCBaseMotion(false);
const bool debug(true);
// isSafe
bool isSafe(true);

int Num_of_Contactpoint = 4;
int dofOfRobot = 18;

Vec41<int> contact_flag_real;
// Functions
void mpcComunicacion();
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void mpcCallback(const ocs2_msgs::mpc_wbc_conversion::ConstPtr& msg);
void gazeboLinkStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
void lf_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg);
void rf_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg);
void lh_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg);
void rh_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg);

int main(int argc, char**argv) {
    //FOR ROS
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle nh;
    ros::Subscriber jointStatesSub, gazeboLinkStatesSub, mpcSub; 
    ros::Subscriber lf_contactState, rf_contactState, lh_contactState, rh_contactState;

    ros::Publisher lf_haa_pub, lf_hfe_pub, lf_kfe_pub;
    ros::Publisher lh_haa_pub, lh_hfe_pub, lh_kfe_pub;
    ros::Publisher rf_haa_pub, rf_hfe_pub, rf_kfe_pub;
    ros::Publisher rh_haa_pub, rh_hfe_pub, rh_kfe_pub;
    

    std_msgs::Float64 lf_haa_tau, lf_hfe_tau, lf_kfe_tau;
    std_msgs::Float64 lh_haa_tau, lh_hfe_tau, lh_kfe_tau;
    std_msgs::Float64 rf_haa_tau, rf_hfe_tau, rf_kfe_tau;
    std_msgs::Float64 rh_haa_tau, rh_hfe_tau, rh_kfe_tau;

    ros::Rate rate(400);

    jointStatesSub = nh.subscribe("/JYPro/joint_states", 1, &jointStatesCallback);
    mpcSub = nh.subscribe("/mpc_wbcPublisher", 1, &mpcCallback);
    gazeboLinkStatesSub = nh.subscribe("/gazebo/model_states", 1,&gazeboLinkStatesCallback);
    lf_contactState = nh.subscribe("/LF_contact", 1, &lf_bumper_callback); 
    rf_contactState = nh.subscribe("/RF_contact", 1, &rf_bumper_callback); 
    lh_contactState = nh.subscribe("/LH_contact", 1, &lh_bumper_callback); 
    rh_contactState = nh.subscribe("/RH_contact", 1, &rh_bumper_callback); 

    lf_haa_pub = nh.advertise<std_msgs::Float64>("/JYPro/LF_HAA_controller/command",1);
    lf_hfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/LF_HFE_controller/command",1);
    lf_kfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/LF_KFE_controller/command",1);
    lh_haa_pub = nh.advertise<std_msgs::Float64>("/JYPro/LH_HAA_controller/command",1);
    lh_hfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/LH_HFE_controller/command",1);
    lh_kfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/LH_KFE_controller/command",1);
    rf_haa_pub = nh.advertise<std_msgs::Float64>("/JYPro/RF_HAA_controller/command",1);
    rf_hfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/RF_HFE_controller/command",1);
    rf_kfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/RF_KFE_controller/command",1);
    rh_haa_pub = nh.advertise<std_msgs::Float64>("/JYPro/RH_HAA_controller/command",1);
    rh_hfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/RH_HFE_controller/command",1);
    rh_kfe_pub = nh.advertise<std_msgs::Float64>("/JYPro/RH_KFE_controller/command",1);
	
	// Simple Motion Control
	simpleMotion.reset(new SimpleMotion(verbose));

    while(nh.ok()){
        // Estimator
        estStatesOutput.time_stamp = ros::Time::now().toSec();
        estStatesOutput.base_pos_world = basePosWorldCur;
        estStatesOutput.base_linear_vel_world = baseLinearVelWorldCur;
        estStatesOutput.base_linear_vel_body = baseLinearVelBodyCur;
        estStatesOutput.base_orientation_world = baseOriWorldCur;
        estStatesOutput.base_angular_vel_world = baseAngularVelWorldCur;
        estStatesOutput.base_angular_vel_body = baseAngularVelBodyCur;

        Eigen::Vector3d baseRpyWorldCur = quaternionTOrpy(baseOriWorldCur);
        // estStatesOutput.frame_c_rpy_in_world << 0, 0, baseRpyWorldCur[2];
        // estStatesOutput.frame_c_quat_in_world = rpyTOquaternion(0., 0., baseRpyWorldCur[2]);
        estStatesOutput.frame_c_rpy_in_world = baseRpyWorldCur;
        estStatesOutput.frame_c_quat_in_world = baseOriWorldCur; //yjy：先试试都转


        estStatesOutput.frame_c_xyz_in_world = basePosWorldCur;


        for(int i(0); i < 3; i++) {
		    estStatesOutput.jointStates.lf_pos.value[i] = jointStatesCur.lf_pos.value[i];
		    estStatesOutput.jointStates.rf_pos.value[i] = jointStatesCur.rf_pos.value[i];
		    estStatesOutput.jointStates.lh_pos.value[i] = jointStatesCur.lh_pos.value[i];
		    estStatesOutput.jointStates.rh_pos.value[i] = jointStatesCur.rh_pos.value[i];

		    estStatesOutput.jointStates.lf_vel.value[i] = jointStatesCur.lf_vel.value[i];
		    estStatesOutput.jointStates.rf_vel.value[i] = jointStatesCur.rf_vel.value[i];
		    estStatesOutput.jointStates.lh_vel.value[i] = jointStatesCur.lh_vel.value[i];
		    estStatesOutput.jointStates.rh_vel.value[i] = jointStatesCur.rh_vel.value[i];	
        }

        simpleMotion->EstimatedStatesInput(estStatesOutput);
	    
        switch (robotState) {
            case kWaitForMsg: {
                lf_haa_tau.data = 0;
                lf_hfe_tau.data = 0;
                lf_kfe_tau.data = 0;

                rf_haa_tau.data = 0;
                rf_hfe_tau.data = 0;
                rf_kfe_tau.data = 0;

                lh_haa_tau.data = 0;
                lh_hfe_tau.data = 0;
                lh_kfe_tau.data = 0;

                rh_haa_tau.data = 0;
                rh_hfe_tau.data = 0;
                rh_kfe_tau.data = 0;

                if(isGazeboMsg && isJointMsg) {
		    		robotState = kPDWaitForStanding;
		    	}
                break;
            }
		    case kPDWaitForStanding: {
		    	if(!isSetUp_PDWaitForStanding) {
		    		simpleMotion->PDSetUpMotion(haa_PDWaitForStanding, hfe_PDWaitForStanding, kfe_PDWaitForStanding, timePDWaitForStanding);
		    		isSetUp_PDWaitForStanding = true;
		    	}
		    	if(simpleMotion->isPDMotionFinished()) {
		    		robotState = kPDStandUpMotion;
		    	}
		    	simpleMotion->PDMotionRun(command);
		    	break;
		    }
		    case kPDStandUpMotion: {
		    	if(!isSetUp_PDStandUpMotion) {
		    		simpleMotion->PDSetUpMotion(haa_PDStandUpMotion, hfe_PDStandUpMotion, kfe_PDStandUpMotion, timePDStandUpMotion);
		    		isSetUp_PDStandUpMotion = true;
		    	}
		    	if(simpleMotion->isPDMotionFinished()) {
		    		robotState = kWBCBaseMotion;
		    	}
		    	simpleMotion->PDMotionRun(command);
		    	break;
		    }
		    case kWBCBaseMotion: {
		    	if(!isSetUp_WBCBaseMotion) {
		    		simpleMotion->WBCSetUpBaseMotion(xBase, yBase, zBase, rollBase, pitchBase, yawBase, timeWBCBaseMotion);                    
                    simpleMotion->UpdateControlFrame(estStatesOutput);
		    		isSetUp_WBCBaseMotion = true;
		    	}
		    	simpleMotion->WBCMotionRun(command, isSafe);
		    	if(simpleMotion->isWBCMotionFinished() && isMPC) {
		    		robotState = kWBCMPC;
		    	}
		    	else if(!isSafe) {
		    		// robotState = kSafeState;
		    	}
		    	break;
		    }
		    case kWBCMPC: {
		    	if(isMPCMsgUpdate) {
		    		// ReadMPCMsg
		    		simpleMotion->UpdateMPCMsg(mpcData, estStatesOutput.time_stamp);
                    simpleMotion->UpdateControlFrame(estStatesOutput);
		    		isMPCMsgUpdate = false;
		    	}
                simpleMotion->TerrainEst(contact_flag_real);
		    	simpleMotion->MPCWBCRun(estStatesOutput.time_stamp, command, isSafe);
		    	if(!isSafe) {
		    		// robotState = kSafeState;
		    	}
		    	break;
		    }
		    case kSafeState: {
		    	if(!isSetUp_SafeState) {
		    		simpleMotion->PDSafeGuardSetUpMotion();
		    		isSetUp_SafeState = true;
		    	}
		    	simpleMotion->PDSafeGuardRun(command);
		    	std::cerr << "\n[dqwang: Motion] kSafeState Now!!!"; 
		    	break;
		    }
        }
        
        lf_haa_tau.data = command.lf_tau.value[0];
        lf_hfe_tau.data = command.lf_tau.value[1];
        lf_kfe_tau.data = command.lf_tau.value[2];

        rf_haa_tau.data = command.rf_tau.value[0];
        rf_hfe_tau.data = command.rf_tau.value[1];
        rf_kfe_tau.data = command.rf_tau.value[2];

        lh_haa_tau.data = command.lh_tau.value[0];
        lh_hfe_tau.data = command.lh_tau.value[1];
        lh_kfe_tau.data = command.lh_tau.value[2];

        rh_haa_tau.data = command.rh_tau.value[0];
        rh_hfe_tau.data = command.rh_tau.value[1];
        rh_kfe_tau.data = command.rh_tau.value[2];

        lf_haa_pub.publish(lf_haa_tau);
        lf_hfe_pub.publish(lf_hfe_tau);
        lf_kfe_pub.publish(lf_kfe_tau);
        
        rf_haa_pub.publish(rf_haa_tau);
        rf_hfe_pub.publish(rf_hfe_tau);
        rf_kfe_pub.publish(rf_kfe_tau);
    
        lh_haa_pub.publish(lh_haa_tau);
        lh_hfe_pub.publish(lh_hfe_tau);
        lh_kfe_pub.publish(lh_kfe_tau);

        rh_haa_pub.publish(rh_haa_tau);
        rh_hfe_pub.publish(rh_hfe_tau);
        rh_kfe_pub.publish(rh_kfe_tau);

        ros::spinOnce();

        bool rate_bool = rate.sleep();
    }

    return 0;
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for(int i(0); i < 3; i++) {
        jointStatesCur.lf_pos.value[i] = msg->position[i];
        jointStatesCur.rf_pos.value[i] = msg->position[i+3];
        jointStatesCur.lh_pos.value[i] = msg->position[i+6];
        jointStatesCur.rh_pos.value[i] = msg->position[i+9];

        jointStatesCur.lf_vel.value[i] = msg->velocity[i];
        jointStatesCur.rf_vel.value[i] = msg->velocity[i+3];
        jointStatesCur.lh_vel.value[i] = msg->velocity[i+6];
        jointStatesCur.rh_vel.value[i] = msg->velocity[i+9];
    }
    isJointMsg = true;
}

void gazeboLinkStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    basePosWorldCur[0] = msg->pose[2].position.x;
    basePosWorldCur[1] = msg->pose[2].position.y;
    basePosWorldCur[2] = msg->pose[2].position.z;

    baseLinearVelWorldCur[0] = msg->twist[2].linear.x; 
    baseLinearVelWorldCur[1] = msg->twist[2].linear.y; 
    baseLinearVelWorldCur[2] = msg->twist[2].linear.z; 


    baseOriWorldCur.w() = msg->pose[2].orientation.w;
    baseOriWorldCur.x() = msg->pose[2].orientation.x;
    baseOriWorldCur.y() = msg->pose[2].orientation.y;
    baseOriWorldCur.z() = msg->pose[2].orientation.z;

    baseAngularVelWorldCur[0] = msg->twist[2].angular.x;
    baseAngularVelWorldCur[1] = msg->twist[2].angular.y;
    baseAngularVelWorldCur[2] = msg->twist[2].angular.z;

    baseLinearVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseLinearVelWorldCur;
    
    baseAngularVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseAngularVelWorldCur; 

    isGazeboMsg = true;
}

void mpcCallback(const ocs2_msgs::mpc_wbc_conversion::ConstPtr& msg) {
    const int N_times = msg->stateTime.size();
    // std::cerr << "N_times: " << N_times << "\n";

    mpcData.stateTime.resize(N_times);
    mpcData.baseAcceleration.clear();
    mpcData.baseAcceleration.resize(N_times);
    mpcData.baseVelocity.clear();
    mpcData.baseVelocity.resize(N_times);
    mpcData.basePosition.clear();
    mpcData.basePosition.resize(N_times);
    mpcData.swingFeetAcceleration.clear();
    mpcData.swingFeetAcceleration.resize(N_times);
    mpcData.swingFeetPosition.clear();
    mpcData.swingFeetPosition.resize(N_times);
    mpcData.swingFeetVelocity.clear();
    mpcData.swingFeetVelocity.resize(N_times);
    mpcData.switchTime.Zero();

    for (int i = 0; i < N_times; i++) {
        // State Times
        mpcData.stateTime[i] = msg->stateTime[i];
        // Swing Feet Trajectories
        mpcData.swingFeetPosition[i].resize(Num_of_Contactpoint);
        mpcData.swingFeetVelocity[i].resize(Num_of_Contactpoint);
        mpcData.swingFeetAcceleration[i].resize(Num_of_Contactpoint);

        for (uint8_t j = 0; j < Num_of_Contactpoint; j++){
            mpcData.swingFeetPosition[i][j].resize(3);
            mpcData.swingFeetVelocity[i][j].resize(3);
            mpcData.swingFeetAcceleration[i][j].resize(3);
        }
        for (int k = 0; k < 3; k++) { 
            // LF
            mpcData.swingFeetPosition[i][0][k]     = (float)msg->wbcTraj[i].swingPos[k];
            mpcData.swingFeetVelocity[i][0][k]     = (float)msg->wbcTraj[i].swingVel[k];
            mpcData.swingFeetAcceleration[i][0][k] = (float)msg->wbcTraj[i].swingAcc[k]; 
            // RF
            mpcData.swingFeetPosition[i][1][k]     = (float)msg->wbcTraj[i].swingPos[k + 3];
            mpcData.swingFeetVelocity[i][1][k]     = (float)msg->wbcTraj[i].swingVel[k + 3];
            mpcData.swingFeetAcceleration[i][1][k] = (float)msg->wbcTraj[i].swingAcc[k + 3];
            // LH
            mpcData.swingFeetPosition[i][2][k]     = (float)msg->wbcTraj[i].swingPos[k + 6];
            mpcData.swingFeetVelocity[i][2][k]     = (float)msg->wbcTraj[i].swingVel[k + 6];
            mpcData.swingFeetAcceleration[i][2][k] = (float)msg->wbcTraj[i].swingAcc[k + 6];    
            // RH
            mpcData.swingFeetPosition[i][3][k]     = (float)msg->wbcTraj[i].swingPos[k + 9];
            mpcData.swingFeetVelocity[i][3][k]     = (float)msg->wbcTraj[i].swingVel[k + 9];
            mpcData.swingFeetAcceleration[i][3][k] = (float)msg->wbcTraj[i].swingAcc[k + 9];    
        }

        mpcData.firstGait[0] = msg->firstGait[0]; // LF
        mpcData.firstGait[1] = msg->firstGait[1]; // RF
        mpcData.firstGait[2] = msg->firstGait[2]; // LH
        mpcData.firstGait[3] = msg->firstGait[3]; // RH

        mpcData.secondGait[0] = msg->secondGait[0]; // LF
        mpcData.secondGait[1] = msg->secondGait[1]; // RF
        mpcData.secondGait[2] = msg->secondGait[2]; // LH
        mpcData.secondGait[3] = msg->secondGait[3]; // RH

        mpcData.thirdGait[0] = msg->thirdGait[0]; // LF
        mpcData.thirdGait[1] = msg->thirdGait[1]; // RF
        mpcData.thirdGait[2] = msg->thirdGait[2]; // LH
        mpcData.thirdGait[3] = msg->thirdGait[3]; // RH

        mpcData.switchTime[0] = msg->switchTime[0]; mpcData.switchTime[1] = msg->switchTime[1]; 

        for (uint8_t i = 0; i < N_times; i++){
            mpcData.basePosition[i].resize(6);
            mpcData.baseVelocity[i].resize(6);
            mpcData.baseAcceleration[i].resize(6);
            for (uint8_t j = 0; j < 6; j++){
                // x y z r p y
                mpcData.basePosition[i][j]     = msg->wbcTraj[i].basePos[j];
                mpcData.baseVelocity[i][j]     = msg->wbcTraj[i].baseVel[j];
                mpcData.baseAcceleration[i][j] = msg->wbcTraj[i].baseAcc[j];
            }
        }
    }
	isMPC = true;
	isMPCMsgUpdate = true;
}


void lf_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    contact_flag_real[0] = int(!msg->states.empty());
}
void rf_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    contact_flag_real[2] = int(!msg->states.empty());
}
void lh_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    contact_flag_real[1] = int(!msg->states.empty());
}
void rh_bumper_callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    contact_flag_real[3] = int(!msg->states.empty());
}

