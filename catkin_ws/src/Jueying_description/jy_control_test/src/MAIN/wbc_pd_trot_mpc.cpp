// #pragma GCC optimize(2)
#include "quadruped_dynamics_model.h"
#include "wbc_ctrl.h"
#include "Gait.h"
#include "UserParameter.h"
#include "cppTypes.h"
#include "utility.h"
#include "wbc_stand_in_jypro.h"

#include <iostream>
#include <fstream>
#include <thread>  

#include "ros/ros.h"
#include "time.h"
#include "Math/utility.h"
#include <ros/node_handle.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "pronto_msgs/QuadrupedStance.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "JYPro_msg/DesLocomotion.h"
#include "JYPro_msg/DesCoM.h"
#include "JYPro_msg/DesLF.h"
#include "JYPro_msg/DesLB.h"
#include "JYPro_msg/DesRF.h"
#include "JYPro_msg/DesRB.h"
#include "JYPro_msg/DesContact.h"
#include "JYPro_msg/DesFootLocation.h"

#include "ocs2_msgs/mpc_wbc_conversion.h"
#include <ocs2_msgs/mpc_wbc_traj.h>


#include <chrono>

using namespace chrono;
using namespace std;

//_____________GLOBAL VALUEABLE___________
//FOR WBC CONTROL
struct conversionData{
    vector_array2_t swingFeetPosition; //lf lh rf rh
    vector_array2_t swingFeetVelocity; //lf lh rf rh
    vector_array2_t swingFeetAcceleration;  //lf lh rf rh
    Eigen::Vector4f firstGait; //lf lh rf rh
    Eigen::Vector4f secondGait; //lf lh rf rh
    Eigen::Vector4f thirdGait; //lf lh rf rh
    Eigen::Vector2f switchTime;
    vector_array_t basePosition;
    vector_array_t baseVelocity;
    vector_array_t baseAcceleration;
    Eigen::VectorXf stateTime;
};
conversionData mpcData;
int Num_of_Contactpoint = 4;
int dofOfRobot = 18;
DVec<float> q(dofOfRobot); 
DVec<float> v(dofOfRobot);

ControlFSMData<float> data;   
LocomotionCtrlData<float> _wbc_data;
DVec<float> tau;
bool joint_wbc_Msg(false);
bool joint_pd_Msg(false);
bool contactMsg(false);
bool poseMsg(false);
bool twistMsg(false);
bool pdDone(false);

int iterpdDone = 0;
 
Eigen::Matrix<float,6,1> sX, sY, sZ, sRoll, sPitch, sYaw;
Eigen::Matrix<float,24,1> sX_all, sY_all;
Eigen::Matrix<float,3,3> rotWorldToBody;
Eigen::Matrix<float,3,3> rotFrame;
float finalTime_com(0.3);
bool desCoM_Msg(false);
bool desLF_Msg(false);
bool desLB_Msg(false);
bool desRB_Msg(false);
bool desRF_Msg(false);
bool desContact_Msg(false);
bool  mpcMsg(false);
bool gazeboState_Msg(false);
bool gazeboFirstIter(true);
float gazeboFirstSateX, gazeboFirstSateY, gazeboFirstSateZ;

size_t iter_mpc(0);
size_t index_state_time(0);//
size_t index_switch_time(0);

float com_x_now, com_y_now, com_z_now;
float com_roll_now, com_pitch_now, com_yaw_now;
Vec31<float> pBody_des, vBody_des, aBody_des;
Vec31<float> pBody_RPY_des, vBody_RPY_des, aBody_RPY_des;
Vec31<float> pFoot_des[4], vFoot_des[4], aFoot_des[4];
Vec2<float> center_0;
Vec2<float> center_now;
UserParameter<float> param;
float yaw_world = 0;
float pitch_world = 0;
float roll_world = 0;
float x_world = 0;
float y_world = 0;
float z_world = 0;
Mat3<float> rotation_world_to_local;
Mat3<float> rotation_local_to_world;
Vec31<float> rpy_world;
Vec31<float> xyz_world;

bool mpcNewMsg = false;

quaternionToRad yawTotalCounter;


long long int iter_com(0);
long long int iter_Spline(0);

bool plan_update(true);
bool if_des_contact(false);


//FOR PD CONTROL
DataLegs data_leg_now;
long long int iter = 0;
long long int iter_1 = 0;
bool pd_0(true);
bool pd_1(false);
bool init_angle(true);

//_____________TRAJECTORY FUNCTION___________
Vec31<double> TrajectoryPlan_d(double startPoint, double finalPoint, double finalTime, double time_traj){
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

Vec31<float> TrajectoryPlan_f(float startPoint, float finalPoint, float finalTime, float time_traj){
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

Eigen::Matrix<float,6,1> TrajectoryPlan(float startPos,
                                        float startVel,
                                        float startAcc,
                                        float finalPos,
                                        float finalVel,
                                        float finalAcc,
                                        float final_time){
    Eigen::Matrix<float,6,1> spline;
    spline[5] = startPos;
    spline[4] = startVel;
    spline[3] = startAcc/2;
    spline[2] = (20*finalPos - 20*startPos - (8*finalVel+12*startVel)*final_time - (3*startAcc-finalAcc)*pow(final_time,2))/(2*pow(final_time,3));
    spline[1] = (30*startPos - 30*finalPos + (14*finalVel+16*startVel)*final_time + (3*startAcc-2*finalAcc)*pow(final_time,2))/(2*pow(final_time,4));
    spline[0] = (12*finalPos - 12*startPos - (6*finalVel+6*startVel)*final_time - (startAcc - finalAcc)*pow(final_time,2))/(2*pow(final_time,5));

    return spline;
}

//________________SUBSCRIBE CALLBACK FUNCTION________________
void joint_state_wbc_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //LF_HAA, LF_HFE, LF_KFE
    for(int i(0); i<3; i++){
        data.legStateEst[legID::LF].q[i] = msg->position[i];
        data.legStateEst[legID::LF].qd[i] = msg->velocity[i];
    }
    //RF_HAA, RF_HFE, RF_KFE
    for(int i(0); i<3; i++){
        data.legStateEst[legID::RF].q[i] = msg->position[i+3];
        data.legStateEst[legID::RF].qd[i] = msg->velocity[i+3];
    }
    //LH_HAA, LH_HFE, LH_KFE
    for(int i(0); i<3; i++){
        data.legStateEst[legID::LB].q[i] = msg->position[i+6];
        data.legStateEst[legID::LB].qd[i] = msg->velocity[i+6];
    }
    //RH_HAA, RH_HFE, RH_KFE
    for(int i(0); i<3; i++){
        data.legStateEst[legID::RB].q[i] = msg->position[i+9];
        data.legStateEst[legID::RB].qd[i] = msg->velocity[i+9];
    }
    joint_wbc_Msg = true;
}

void joint_state_pd_callback(const sensor_msgs::JointState::ConstPtr& msg){
    std::cout<<"3";
    for(int i(0); i<3; i++){
        data_leg_now.fl_pos.value[i] = msg->position[i];
        data_leg_now.fr_pos.value[i] = msg->position[i+3];
        data_leg_now.hl_pos.value[i] = msg->position[i+6];
        data_leg_now.hr_pos.value[i] = msg->position[i+9];

        data_leg_now.fl_vel.value[i] = msg->velocity[i];
        data_leg_now.fr_vel.value[i] = msg->velocity[i+3];
        data_leg_now.hl_vel.value[i] = msg->velocity[i+6];
        data_leg_now.hr_vel.value[i] = msg->velocity[i+9];

        data.legStateEst[legID::LF].q[i] = msg->position[i];
        data.legStateEst[legID::LF].qd[i] = msg->velocity[i];
        data.legStateEst[legID::RF].q[i] = msg->position[i+3];
        data.legStateEst[legID::RF].qd[i] = msg->velocity[i+3];
        data.legStateEst[legID::LB].q[i] = msg->position[i+6];
        data.legStateEst[legID::LB].qd[i] = msg->velocity[i+6];
        data.legStateEst[legID::RB].q[i] = msg->position[i+9];
        data.legStateEst[legID::RB].qd[i] = msg->velocity[i+9];
    }
    joint_pd_Msg = true;
}

void contact_state_callback(const pronto_msgs::QuadrupedStance::ConstPtr& msg)
{
    data.bodyStateEst.contactEstimate[legID::LF] = msg->lf;
    data.bodyStateEst.contactEstimate[legID::LB] = msg->lh;
    data.bodyStateEst.contactEstimate[legID::RF] = msg->rf;
    data.bodyStateEst.contactEstimate[legID::RB] = msg->rh;

    contactMsg = true;
}

void pose_state_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // data.bodyStateEst.position[0] = msg->pose.pose.position.x;
    // data.bodyStateEst.position[1] = msg->pose.pose.position.y;
    // data.bodyStateEst.position[2] = msg->pose.pose.position.z;

    // data.bodyStateEst.orientation.w() = msg->pose.pose.orientation.w;
    // data.bodyStateEst.orientation.x() = msg->pose.pose.orientation.x;
    // data.bodyStateEst.orientation.y() = msg->pose.pose.orientation.y;
    // data.bodyStateEst.orientation.z() = msg->pose.pose.orientation.z;
    // data.bodyStateEst.rpy = quaternionTOrpy(data.bodyStateEst.orientation).cast<float>();
    // data.bodyStateEst.RotationMat = rpyTORotateMat(data.bodyStateEst.rpy[0],
    //                                                data.bodyStateEst.rpy[1],
    //                                                data.bodyStateEst.rpy[2]);
    
    poseMsg = true;
}

void twist_state_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
    // data.bodyStateEst.vBody[0] = msg->twist.twist.linear.x;
    // data.bodyStateEst.vBody[1] = msg->twist.twist.linear.y;
    // data.bodyStateEst.vBody[2] = msg->twist.twist.linear.z - 0.009;

    // data.bodyStateEst.omegaBody[0] = msg->twist.twist.angular.x;
    // data.bodyStateEst.omegaBody[1] = msg->twist.twist.angular.y;
    // data.bodyStateEst.omegaBody[2] = msg->twist.twist.angular.z;

    twistMsg = true;
}



void mpc_output_callback(const ocs2_msgs::mpc_wbc_conversion::ConstPtr& msg){

    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


    cout << "I GET THE MPC_WBC_MSG!" << endl;

    const int N_times = msg->stateTime.size();
    cout << "Size:" << N_times << "\n";

    // //****************************************************************
    // mpcData.stateTime.resize(N_times);
    // mpcData.baseAcceleration.clear();
    // mpcData.baseAcceleration.resize(N_times);
    // mpcData.baseVelocity.clear();
    // mpcData.baseVelocity.resize(N_times);
    // mpcData.basePosition.clear();
    // mpcData.basePosition.resize(N_times);
    // mpcData.swingFeetAcceleration.clear();
    // mpcData.swingFeetAcceleration.resize(N_times);
    // mpcData.swingFeetPosition.clear();
    // mpcData.swingFeetPosition.resize(N_times);
    // mpcData.swingFeetVelocity.clear();
    // mpcData.swingFeetVelocity.resize(N_times);
    // mpcData.switchTime.Zero();

    // std::vector<double> stateTimeVectorTpl(msg->stateTime);
    // mpcData.stateTime = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(stateTimeVectorTpl.data(), stateTimeVectorTpl.size()).cast<float>();

    // std::cout << "vector i got: " << mpcData.stateTime << "\n";
    // int count = 0;
     
    // for (auto eachTrajectory : msg->wbcTraj){
    //     mpcData.basePosition[count]     =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(eachTrajectory.q.data(), eachTrajectory.q.size()).cast<float>();
    //     mpcData.baseVelocity[count]     =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(eachTrajectory.qdot.data(), eachTrajectory.qdot.size()).cast<float>();
    //     mpcData.baseAcceleration[count] =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(eachTrajectory.qdotdot.data(), eachTrajectory.qdotdot.size()).cast<float>();

    //     mpcData.swingFeetPosition[count] = eachTrajectory.swingPos;


    //     ++count;
    // }
    // //*****************************************************************

    




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

    // cout<< "swingPos:_______\n" << msg->wbcTraj[0].swingPos.size() << "\n________________________________________\n";
    // cout<< "swingVel:_______\n" << msg->wbcTraj[0].swingVel.size() << "\n________________________________________\n";
    // cout<< "swingAcc:_______\n" << msg->wbcTraj[0].swingAcc.size() << "\n________________________________________\n";

    for (int16_t i = 0; i < N_times; i++){
        mpcData.swingFeetPosition[i].resize(Num_of_Contactpoint);
        mpcData.swingFeetVelocity[i].resize(Num_of_Contactpoint);
        mpcData.swingFeetAcceleration[i].resize(Num_of_Contactpoint);
    }
    for (int16_t i = 0; i < N_times; i++){
        mpcData.stateTime[i] = msg->stateTime[i];

        for (uint8_t j = 0; j < Num_of_Contactpoint; j++){
            mpcData.swingFeetPosition[i][j].resize(3);
            mpcData.swingFeetVelocity[i][j].resize(3);
            mpcData.swingFeetAcceleration[i][j].resize(3);
        }
        for(uint8_t k = 0; k < 3; k++){
            //lf lh rf rh                        //lf rf lh rh
            mpcData.swingFeetPosition[i][0][k] = (float)msg->wbcTraj[i].swingPos[k];
            mpcData.swingFeetVelocity[i][0][k] = (float)msg->wbcTraj[i].swingVel[k];
            mpcData.swingFeetAcceleration[i][0][k] = (float)msg->wbcTraj[i].swingAcc[k]; 
            // cout << "\n2________\n";


            mpcData.swingFeetPosition[i][2][k] = (float)msg->wbcTraj[i].swingPos[k + 3];
            mpcData.swingFeetVelocity[i][2][k] = (float)msg->wbcTraj[i].swingVel[k + 3];
            mpcData.swingFeetAcceleration[i][2][k] = (float)msg->wbcTraj[i].swingAcc[k + 3]; 
            // cout << "\n3________\n";


            mpcData.swingFeetPosition[i][1][k] = (float)msg->wbcTraj[i].swingPos[k + 6];
            mpcData.swingFeetVelocity[i][1][k] = (float)msg->wbcTraj[i].swingVel[k + 6];
            mpcData.swingFeetAcceleration[i][1][k] = (float)msg->wbcTraj[i].swingAcc[k + 6]; 

            // cout << "\n4________\n";

            mpcData.swingFeetPosition[i][3][k] = (float)msg->wbcTraj[i].swingPos[k + 9];
            mpcData.swingFeetVelocity[i][3][k] = (float)msg->wbcTraj[i].swingVel[k + 9];
            mpcData.swingFeetAcceleration[i][3][k] = (float)msg->wbcTraj[i].swingAcc[k + 9];
        }
    }

    for (uint8_t i = 0; i < 4; i++){
        //lf rf lh rh
        mpcData.firstGait[i] = msg->firstGait[i];
        mpcData.secondGait[i] = msg->secondGait[i];
        mpcData.thirdGait[i] = msg->thirdGait[i];
    }

    mpcData.switchTime[0] = msg->switchTime[0]; mpcData.switchTime[1] = msg->switchTime[1]; 

    for (uint8_t i = 0; i < N_times; i++){
        mpcData.basePosition[i].resize(6);
        mpcData.baseVelocity[i].resize(6);
        mpcData.baseAcceleration[i].resize(6);
        for (uint8_t j = 0; j < 6; j++){
            // x y z yaw pitch roll
            mpcData.basePosition[i][j] = msg->wbcTraj[i].basePos[j];
            mpcData.baseVelocity[i][j] = msg->wbcTraj[i].baseVel[j];
            mpcData.baseAcceleration[i][j] = msg->wbcTraj[i].baseAcc[j];
        }
    }

    // cout<< "swingPos:_______\n" << msg->wbcTraj[0].swingPos.size() << "\n________________________________________\n";
    // cout<< "swingVel:_______\n" << msg->wbcTraj[0].swingVel.size() << "\n________________________________________\n";
    // cout<< "swingAcc:_______\n" << msg->wbcTraj[0].swingAcc.size() << "\n________________________________________\n";
    // cout<< "switchtime:_______\n" << mpcData.switchTime << "\n________________________________________\n";
    // cout<< "q0:_________________\n" << mpcData.basePosition[0] << "\n________________________________________\n";
    // cout<< "qdot0:_________________\n" << mpcData.baseVelocity[0] << "\n________________________________________\n";
    // cout<< "qdotdot0:_________________\n" << mpcData.baseAcceleration[0] << "\n________________________________________\n";

    // cout<< "q1:_________________\n" << mpcData.basePosition[1] << "\n________________________________________\n";


    mpcMsg = true;
    iter_mpc = 0;
    index_state_time = 0;
    index_switch_time = 0;

    mpcNewMsg = true;


    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cerr << "time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;
}

void gazebo_link_states_callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    // pose --- orientation
    std::cout<<"2";
    double ros_time = ros::Time::now().toSec(); 
    Eigen::Quaternion<float> orientation_world, orientation_tpl, orientation_local;
    
    orientation_world.w() = msg->pose[2].orientation.w;
    orientation_world.x() = msg->pose[2].orientation.x;
    orientation_world.y() = msg->pose[2].orientation.y;
    orientation_world.z() = msg->pose[2].orientation.z;

    if(mpcNewMsg){
        rpy_world = yawTotalCounter.quaternionToTotalRad(orientation_world);
        std::cout << "\n>>>>>>>>>>>>>>>>rpy_world:" << rpy_world[2] << "\n";
        // rpy_world = quaternionTOrpy(orientation_world);
        yaw_world = rpy_world[2];
        pitch_world = rpy_world[1];
        roll_world = rpy_world[0];

        x_world = msg->pose[2].position.x;
        y_world = msg->pose[2].position.y;  
        z_world = msg->pose[2].position.z;

        xyz_world << x_world, y_world, z_world;
        mpcNewMsg = false;
    }

    rotation_world_to_local = rpyTORotateMat(roll_world, pitch_world, yaw_world).transpose();
    // rotation_local_to_world = rpyTORotateMat(0.0f, 0.0f, yaw_world);

    orientation_tpl = rotation_world_to_local;   

    orientation_local = orientation_tpl * orientation_world;
    data.bodyStateEst.rpy = quaternionTOrpy(orientation_local);
    data.bodyStateEst.RotationMat = rpyTORotateMat(data.bodyStateEst.rpy[0],
                                                   data.bodyStateEst.rpy[1],
                                                   data.bodyStateEst.rpy[2]);                               
    data.bodyStateEst.orientation.w() = orientation_local.w();
    data.bodyStateEst.orientation.x() = orientation_local.x();
    data.bodyStateEst.orientation.y() = orientation_local.y();
    data.bodyStateEst.orientation.z() = orientation_local.z();
    // Mat3<float> rot_initial, rot_now, rot_err;
    // Vec31<float> rpy_now;
    // rpy_now = quaternionTOrpy(orientation_world);
    // rot_initial = rpyTORotateMat(0.0f, 0.0f, yaw_world);
    // rot_now = rpyTORotateMat(rpy_now[0], rpy_now[1], rpy_now[2]);
    // rot_err = rot_now * rot_initial.transpose();
    // data.bodyStateEst.rpy[0] = rot_err(2,1);
    // data.bodyStateEst.rpy[1] = rot_err(0,2);
    // data.bodyStateEst.rpy[2] = rot_err(1,0);
    // data.bodyStateEst.RotationMat = rpyTORotateMat(data.bodyStateEst.rpy[0],
    //                                                data.bodyStateEst.rpy[1],
    //                                                data.bodyStateEst.rpy[2]);
    // data.bodyStateEst.orientation = data.bodyStateEst.RotationMat;

    // pose --- position
    Vec31<float> position;
    position[0] = msg->pose[2].position.x;
    position[1] = msg->pose[2].position.y;
    position[2] = msg->pose[2].position.z;
    data.bodyStateEst.position = rotation_world_to_local * (position - xyz_world);

    // twist --- angular
    Vec31<float> omegaBody;
    omegaBody[0] = msg->twist[2].angular.x;
    omegaBody[1] = msg->twist[2].angular.y;
    omegaBody[2] = msg->twist[2].angular.z;
    data.bodyStateEst.omegaBody = rotation_world_to_local * omegaBody;

    // twist --- linear
    Vec31<float> vBody;
    vBody[0] = msg->twist[2].linear.x;
    vBody[1] = msg->twist[2].linear.y;
    vBody[2] = msg->twist[2].linear.z;
    data.bodyStateEst.vBody = rotation_world_to_local * vBody;

    gazeboState_Msg = true;
}

int main(int argc, char**argv){
    //FOR ROS
    ros::init(argc, argv, "jy_wbc_node");
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_wbc_, joint_state_pd_, contact_state_, pose_, twist_;
    ros::Subscriber des_CoM, des_LF, des_LB, des_RF, des_RB, des_Contact, des_footholds, mpc_output;
    ros::Subscriber gazebo_linkStates;
    ros::Publisher lf_haa, lf_hfe, lf_kfe, lb_haa, lb_hfe, lb_kfe, rf_haa, rf_hfe, rf_kfe, rb_haa, rb_hfe, rb_kfe;
    ros::Publisher planner_on;

    std_msgs::Float64 tau_lf_haa, tau_lf_hfe, tau_lf_kfe, tau_lb_haa, tau_lb_hfe, tau_lb_kfe;
    std_msgs::Float64 tau_rf_haa, tau_rf_hfe, tau_rf_kfe, tau_rb_haa, tau_rb_hfe, tau_rb_kfe;
    std_msgs::Float64 if_plan;
    if_plan.data = 0.0;

    ros::Rate rate(400);
    double cycle_time = 0.0025;

    ros::AsyncSpinner spinner(0);

    // joint_state_wbc_ = nh_.subscribe("/state_estimator_pronto/prior_joint_accel", 10, &joint_state_wbc_callback);
    joint_state_pd_ = nh_.subscribe("/JYPro/joint_states", 1, &joint_state_pd_callback);
    // contact_state_ = nh_.subscribe("/state_estimator_pronto/stance", 10, &contact_state_callback);
    // pose_ = nh_.subscribe("/state_estimator_pronto/pose", 10, &pose_state_callback);
    // twist_ = nh_.subscribe("/state_estimator_pronto/twist",10, &twist_state_callback);
    mpc_output = nh_.subscribe("/mpc_wbcPublisher", 1, &mpc_output_callback);
    gazebo_linkStates = nh_.subscribe("/gazebo/link_states", 1,&gazebo_link_states_callback);



    lf_haa = nh_.advertise<std_msgs::Float64>("/JYPro/LF_HAA_controller/command",1);
    lf_hfe = nh_.advertise<std_msgs::Float64>("/JYPro/LF_HFE_controller/command",1);
    lf_kfe = nh_.advertise<std_msgs::Float64>("/JYPro/LF_KFE_controller/command",1);
    lb_haa = nh_.advertise<std_msgs::Float64>("/JYPro/LH_HAA_controller/command",1);
    lb_hfe = nh_.advertise<std_msgs::Float64>("/JYPro/LH_HFE_controller/command",1);
    lb_kfe = nh_.advertise<std_msgs::Float64>("/JYPro/LH_KFE_controller/command",1);
    rf_haa = nh_.advertise<std_msgs::Float64>("/JYPro/RF_HAA_controller/command",1);
    rf_hfe = nh_.advertise<std_msgs::Float64>("/JYPro/RF_HFE_controller/command",1);
    rf_kfe = nh_.advertise<std_msgs::Float64>("/JYPro/RF_KFE_controller/command",1);
    rb_haa = nh_.advertise<std_msgs::Float64>("/JYPro/RH_HAA_controller/command",1);
    rb_hfe = nh_.advertise<std_msgs::Float64>("/JYPro/RH_HFE_controller/command",1);
    rb_kfe = nh_.advertise<std_msgs::Float64>("/JYPro/RH_KFE_controller/command",1);
    planner_on = nh_.advertise<std_msgs::Float64>("/planner_switch",1);

    // spinner.start();
    //FOR WBC
    UserParameter<float> paramf;
    QuadrupedDynamicsModel jueying;
    WBC_Ctrl<float> _wbc_ctrl(&jueying);
    BodyPositionEst _bodyEst;

    long long int Number(0);
    long long int wbc_iter(0);
    float reset_pose_time(2);
    float des_y(0.00);

    float lf_x, lb_x, rf_x, rb_x;

    ofstream in_x, in_y, in_z;
    ofstream in_roll, in_pitch, in_yaw;
    ofstream in_x_vel, in_y_vel, in_z_vel;
    ofstream in_roll_vel, in_pitch_vel, in_yaw_vel;
    ofstream in_foot_lf_x, in_foot_lh_x, in_foot_rf_x, in_foot_rh_x;  
    ofstream in_foot_lf_y, in_foot_lh_y, in_foot_rf_y, in_foot_rh_y;  
    ofstream in_foot_lf_z, in_foot_lh_z, in_foot_rf_z, in_foot_rh_z;  

    ofstream in_foot_lf_x_vel, in_foot_lh_x_vel, in_foot_rf_x_vel, in_foot_rh_x_vel;  
    ofstream in_foot_lf_y_vel, in_foot_lh_y_vel, in_foot_rf_y_vel, in_foot_rh_y_vel;  
    ofstream in_foot_lf_z_vel, in_foot_lh_z_vel, in_foot_rf_z_vel, in_foot_rh_z_vel;  

    ofstream in_vel_lf_z;
    ofstream in_acc_lf_z;

    ofstream in_contact_mpc;
    if(true){
    in_x.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_x.txt", ios::trunc);
    in_y.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_y.txt", ios::trunc);
    in_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_z.txt", ios::trunc);
    in_roll.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_roll.txt", ios::trunc);
    in_pitch.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_pitch.txt", ios::trunc);
    in_yaw.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_yaw.txt", ios::trunc);

    in_x_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_x_vel.txt", ios::trunc);
    in_y_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_y_vel.txt", ios::trunc);
    in_z_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_z_vel.txt", ios::trunc);
    in_roll_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_roll_vel.txt", ios::trunc);
    in_pitch_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_pitch_vel.txt", ios::trunc);
    in_yaw_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_yaw_vel.txt", ios::trunc);

    in_foot_lf_x.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_x.txt", ios::trunc);
    in_foot_lh_x.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_x.txt", ios::trunc);
    in_foot_rf_x.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_x.txt", ios::trunc);
    in_foot_rh_x.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_x.txt", ios::trunc);

    in_foot_lf_y.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_y.txt", ios::trunc);
    in_foot_lh_y.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_y.txt", ios::trunc);
    in_foot_rf_y.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_y.txt", ios::trunc);
    in_foot_rh_y.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_y.txt", ios::trunc);

    in_foot_lf_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_z.txt", ios::trunc);
    in_foot_lh_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_z.txt", ios::trunc);
    in_foot_rf_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_z.txt", ios::trunc);
    in_foot_rh_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_z.txt", ios::trunc);

    in_foot_lf_x_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_x_vel.txt", ios::trunc);
    in_foot_lh_x_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_x_vel.txt", ios::trunc);
    in_foot_rf_x_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_x_vel.txt", ios::trunc);
    in_foot_rh_x_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_x_vel.txt", ios::trunc);

    in_foot_lf_y_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_y_vel.txt", ios::trunc);
    in_foot_lh_y_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_y_vel.txt", ios::trunc);
    in_foot_rf_y_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_y_vel.txt", ios::trunc);
    in_foot_rh_y_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_y_vel.txt", ios::trunc);

    in_foot_lf_z_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_z_vel.txt", ios::trunc);
    in_foot_lh_z_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lh_z_vel.txt", ios::trunc);
    in_foot_rf_z_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rf_z_vel.txt", ios::trunc);
    in_foot_rh_z_vel.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_rh_z_vel.txt", ios::trunc);

    in_vel_lf_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_velz.txt", ios::trunc);
    in_acc_lf_z.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_foot_lf_accz.txt", ios::trunc);
    
    in_contact_mpc.open("/home/yjy/MPC_WBC_ori/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/wbc_analysis_contact.txt", ios::trunc);
    }
    while(nh_.ok()){
        std::cout<<"1";
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        if(joint_pd_Msg){
            Number++;
            if((Number)&&(gazeboState_Msg)&&(pdDone)){ // WBC
                static const bool usingModel = true;
                wbc_iter++;
                // data.bodyStateEst.position = _bodyEst.bodyPositionEst(data.legStateEst[legID::LF].q,
                //                                                       data.legStateEst[legID::LB].q,
                //                                                       data.legStateEst[legID::RF].q,
                //                                                       data.legStateEst[legID::RB].q,
                //                                                       data.bodyStateEst.contactEstimate
                //                                                       );
                if(wbc_iter == 1){
                    com_x_now = data.bodyStateEst.position[0];
                    com_y_now = data.bodyStateEst.position[1];
                    com_z_now = data.bodyStateEst.position[2];
                    com_roll_now = data.bodyStateEst.rpy[0];
                    com_pitch_now = data.bodyStateEst.rpy[1];
                    com_yaw_now = data.bodyStateEst.rpy[2];

                    sX = TrajectoryPlan(com_x_now, 0, 0, com_x_now, 0, 0, reset_pose_time);
                    sY = TrajectoryPlan(com_y_now, 0, 0, com_y_now - 0.05, 0, 0, reset_pose_time);
                    sZ = TrajectoryPlan(com_z_now, 0, 0, param.body_height, 0, 0, reset_pose_time);
                    sRoll = TrajectoryPlan(com_roll_now, 0, 0, com_roll_now,0, 0, reset_pose_time);
                    sPitch = TrajectoryPlan(com_pitch_now, 0, 0, com_pitch_now,0, 0, reset_pose_time);
                    sYaw = TrajectoryPlan(com_yaw_now, 0, 0, com_yaw_now ,0, 0, reset_pose_time);

                    lf_x = _bodyEst._foot_lf_in_center[0];
                    lb_x = _bodyEst._foot_lb_in_center[0];
                    rf_x = _bodyEst._foot_rf_in_center[0];
                    rb_x = _bodyEst._foot_rb_in_center[0];
                }

                float time_com = iter_com * cycle_time;
                Eigen::Matrix<float,1,6> t, td, tdd;
                t << pow(time_com,5), pow(time_com,4),pow(time_com,3),pow(time_com,2),time_com,1;
                td << 5*pow(time_com,4), 4*pow(time_com,3), 3*pow(time_com,2), 2*time_com, 1,0;
                tdd << 20*pow(time_com,3),12* pow(time_com,2), 6*time_com, 2 ,0,0;

                if(mpcMsg){
                // if(false){

                    // Contact State
                    double ros_time = ros::Time::now().toSec();
                    std::cout << "ros time: " << ros_time << "s\n";
                    std::cout << "traj start time: " << mpcData.stateTime[0] << "s\n";



                    //------------------------------------------------
                    // if(iter_mpc * cycle_time > mpcData.stateTime[index_state_time]){
                    //     index_state_time ++;
                    // }
                    // if(iter_mpc * cycle_time < mpcData.switchTime[0]){
                    //     data.bodyStateEst.contactEstimate = mpcData.firstGait;
                    //     _wbc_data.contact_state = mpcData.firstGait;
                    // }
                    // else if(iter_mpc * cycle_time < mpcData.switchTime[1] && iter_mpc *cycle_time >= mpcData.switchTime[0]){
                    //     data.bodyStateEst.contactEstimate = mpcData.secondGait;
                    //     _wbc_data.contact_state = mpcData.secondGait;                
                    // }
                    // else{
                    //     data.bodyStateEst.contactEstimate = mpcData.thirdGait;
                    //     _wbc_data.contact_state = mpcData.thirdGait;
                    // }
                    //------------------------------------------------
                    index_state_time = 0;
                    while (ros_time > mpcData.stateTime[index_state_time] && index_state_time < mpcData.stateTime.size()){
                        index_state_time ++;
                    }
                    std::cout << "index_state_time: " << index_state_time << "\n";
                    std::cout << "traj index time: " << mpcData.stateTime[index_state_time] << "s\n";
                    std::cout <<  "mpcData.switchTime: " << mpcData.switchTime << "\n";

                    if(ros_time < mpcData.switchTime[0]){
                        data.bodyStateEst.contactEstimate = mpcData.firstGait;
                        _wbc_data.contact_state = mpcData.firstGait;
                    }
                    else if(ros_time < mpcData.switchTime[1] && ros_time >= mpcData.switchTime[0]){
                        data.bodyStateEst.contactEstimate = mpcData.secondGait;
                        _wbc_data.contact_state = mpcData.secondGait;   
                        std::cout << "################################[secondGait]################################\n";             
                    }
                    else{
                        data.bodyStateEst.contactEstimate = mpcData.thirdGait;
                        _wbc_data.contact_state = mpcData.thirdGait;
                        std::cout << "################################[thirdGait]################################\n";             

                    }
                    
                    // data.bodyStateEst.contactEstimate = mpcData.firstGait;
                    // _wbc_data.contact_state = mpcData.firstGait;

                    // data.bodyStateEst.contactEstimate <<  1., 1., 1., 1.;
                    // _wbc_data.contact_state <<  1., 1., 1., 1.;
                    
                    if(index_state_time  >= mpcData.stateTime.size()){
                        index_state_time = mpcData.stateTime.size() - 1;
                        std::cout << "________________________out of time________________________\n";

                        // ROS_INFO("________________________");
                    }
                    // std::cout << "index:" << index_state_time << "\n";
                    // std::cout << "traj to do time:" << mpcData.stateTime[index_state_time] << "s\n";

                        
                    // cout << index_state_time << endl;

                    // Swing leg 
                    // LF
                    pFoot_des[0] = mpcData.swingFeetPosition[index_state_time][0];
                    vFoot_des[0] = mpcData.swingFeetVelocity[index_state_time][0];
                    aFoot_des[0] = mpcData.swingFeetAcceleration[index_state_time][0];
                    // LH
                    pFoot_des[1] = mpcData.swingFeetPosition[index_state_time][1];
                    vFoot_des[1] = mpcData.swingFeetVelocity[index_state_time][1];
                    aFoot_des[1] = mpcData.swingFeetAcceleration[index_state_time][1];
                    // RF
                    pFoot_des[2] = mpcData.swingFeetPosition[index_state_time][2];
                    vFoot_des[2] = mpcData.swingFeetVelocity[index_state_time][2];
                    aFoot_des[2] = mpcData.swingFeetAcceleration[index_state_time][2];
                    //RH
                    pFoot_des[3] = mpcData.swingFeetPosition[index_state_time][3];
                    vFoot_des[3] = mpcData.swingFeetVelocity[index_state_time][3];
                    aFoot_des[3] = mpcData.swingFeetAcceleration[index_state_time][3];

                    pBody_des[0] = mpcData.basePosition[index_state_time][0]; // x position
                    pBody_des[1] = mpcData.basePosition[index_state_time][1]; // y position
                    pBody_des[2] = mpcData.basePosition[index_state_time][2]; // z position

                    pBody_RPY_des[0] = mpcData.basePosition[index_state_time][5]; // roll positon
                    pBody_RPY_des[1] = mpcData.basePosition[index_state_time][4]; // pitch positon
                    pBody_RPY_des[2] = mpcData.basePosition[index_state_time][3]; // yaw positon

                    vBody_des[0] = mpcData.baseVelocity[index_state_time][0]; // x velocity 
                    vBody_des[1] = mpcData.baseVelocity[index_state_time][1]; // y velocity 
                    vBody_des[2] = mpcData.baseVelocity[index_state_time][2]; // z velocity 

                    //小心 如果这里MPC传过来的是角速度 就不用换了
                    
                    vBody_RPY_des[0] = mpcData.baseVelocity[index_state_time][3]; // roll velocity
                    vBody_RPY_des[1] = mpcData.baseVelocity[index_state_time][4]; // pitch velocity
                    vBody_RPY_des[2] = mpcData.baseVelocity[index_state_time][5]; // yaw velocity

                    aBody_des[0] = mpcData.baseAcceleration[index_state_time][0]; // x acceleration
                    aBody_des[1] = mpcData.baseAcceleration[index_state_time][1]; // y acceleration
                    aBody_des[2] = mpcData.baseAcceleration[index_state_time][2]; // z acceleration
                    aBody_RPY_des[0] = mpcData.baseAcceleration[index_state_time][5]; // roll acceleration
                    aBody_RPY_des[1] = mpcData.baseAcceleration[index_state_time][4]; // pitch acceleration
                    aBody_RPY_des[2] = mpcData.baseAcceleration[index_state_time][3]; // yaw acceleration

                    _wbc_data.pBody_des =  rotation_world_to_local * (pBody_des - xyz_world);
                    _wbc_data.vBody_des =  rotation_world_to_local * vBody_des;
                    _wbc_data.aBody_des =  rotation_world_to_local * aBody_des;

                    // Mat3<float> rot_initial, rot_des, rot_err;
                    // rot_initial = rpyTORotateMat(0.0f, 0.0f, yaw_world);
                    // rot_des = rpyTORotateMat(pBody_RPY_des[0], pBody_RPY_des[1], pBody_RPY_des[2]);
                    // rot_err = rot_des * rot_initial.transpose();
                    // _wbc_data.pBody_RPY_des[0] = rot_err(2,1);
                    // _wbc_data.pBody_RPY_des[1] = rot_err(0,2);
                    // _wbc_data.pBody_RPY_des[2] = rot_err(1,0);
                    Eigen::Quaternion<float> orientation_des, orientation_tpl, orientation_local_des;
                    orientation_des = rpyTORotateMat(pBody_RPY_des[0], pBody_RPY_des[1], pBody_RPY_des[2]);
                    orientation_tpl = rotation_world_to_local;
                    orientation_local_des = orientation_tpl * orientation_des;
                    _wbc_data.pBody_RPY_des = quaternionTOrpy(orientation_local_des);
                    // _wbc_data.vBody_RPY_des = rotation_world_to_local * vBody_RPY_des;
                    // _wbc_data.aBody_RPY_des = rotation_world_to_local * aBody_RPY_des; //yujiyu : Needn't orientate angular acc and vel //这有问题啊
                    _wbc_data.vBody_RPY_des = rotation_world_to_local * vBody_RPY_des;
                    _wbc_data.aBody_RPY_des = aBody_RPY_des;//Dont Care Acc of angluar

                    // for(uint i = 0; i < 4; i++){
                    //     _wbc_data.pFoot_des[i] = rotation_world_to_local * (pFoot_des[i] - xyz_world);
                    //     _wbc_data.vFoot_des[i] = rotation_world_to_local * vFoot_des[i];
                    //     _wbc_data.aFoot_des[i] = rotation_world_to_local * aFoot_des[i];
                    // }

                    for(uint8_t i = 0; i < 4; i++){
                        _wbc_data.pFoot_des[i] = pFoot_des[i];
                        _wbc_data.vFoot_des[i] = rotation_world_to_local * vFoot_des[i];
                        _wbc_data.aFoot_des[i] = rotation_world_to_local * aFoot_des[i];
                    }
                

                    iter_mpc++;

                }
                else{
                    pBody_des[0] = t*sX;
                    pBody_des[1] = t*sY;
                    pBody_des[2] = t*sZ;

                    vBody_des[0] = td*sX;
                    vBody_des[1] = td*sY;
                    vBody_des[2] = td*sZ;  

                    aBody_des[0] = tdd*sX;
                    aBody_des[1] = tdd*sY;
                    aBody_des[2] = tdd*sZ; 

                    pBody_RPY_des << t*sRoll, t*sPitch, t*sYaw;
                    vBody_RPY_des << td*sRoll, td*sPitch, td*sYaw;
                    aBody_RPY_des << tdd*sRoll, tdd*sPitch, tdd*sYaw;

                    if(time_com >= reset_pose_time){
                        pBody_des[0] = com_x_now;
                        pBody_des[1] = com_y_now - 0.05;
                        pBody_des[2] = param.body_height;

                        vBody_des[0] = 0.0;
                        vBody_des[1] = 0.0;
                        vBody_des[2] = 0.0;  
    
                        aBody_des[0] = 0.0;
                        aBody_des[1] = 0.0;
                        aBody_des[2] = 0.0; 

                        pBody_RPY_des << com_roll_now, com_pitch_now, com_yaw_now;
                        vBody_RPY_des << 0,0,0;
                        aBody_RPY_des << 0,0,0;
                    }

                    data.bodyStateEst.contactEstimate << 1,1,1,1;
                    _wbc_data.contact_state << 1,1,1,1;

                    _wbc_data.pBody_des = pBody_des;
                    _wbc_data.vBody_des = vBody_des;
                    _wbc_data.aBody_des = aBody_des;

                    _wbc_data.pBody_RPY_des = pBody_RPY_des;
                    _wbc_data.vBody_RPY_des = vBody_RPY_des;
                    _wbc_data.aBody_RPY_des = aBody_RPY_des;

                    for(uint i = 0; i < 4; i++){
                        _wbc_data.pFoot_des[i] = pFoot_des[i];
                        _wbc_data.vFoot_des[i] = vFoot_des[i];
                        _wbc_data.aFoot_des[i] = aFoot_des[i];
                    }
                }
                
                std::chrono::steady_clock::time_point wbct1 = std::chrono::steady_clock::now();
                _wbc_ctrl.run(&_wbc_data, data,tau);
                std::chrono::steady_clock::time_point wbct2 = std::chrono::steady_clock::now();
                // std::cerr << "wbc time:" << std::chrono::duration_cast<std::chrono::microseconds>(wbct2 - wbct1).count() << "us" << std::endl;
                
                tau_lf_haa.data = tau[0]; tau_lf_hfe.data = tau[1]; tau_lf_kfe.data = tau[2]; tau_lb_haa.data=tau[3]; tau_lb_hfe.data=tau[4]; tau_lb_kfe.data=tau[5];
                tau_rf_haa.data = tau[6]; tau_rf_hfe.data = tau[7]; tau_rf_kfe.data = tau[8]; tau_rb_haa.data=tau[9]; tau_rb_hfe.data=tau[10]; tau_rb_kfe.data=tau[11];

                //PRINT
                if(true)
                // if(wbc_iter && mpcMsg)
                {
                    // std::cout << "\n" << std::endl;
                    // std::cout << "WBC_TAU: " << std::endl;
                    // for(int i(0); i<12; i++){
                    //     std::cout << tau[i] << std::endl;  
                    // }
                    // std::cout << "\n" << std::endl;

                    cout << "body position: " << endl;
                    cout << data.bodyStateEst.position[0] << " " << data.bodyStateEst.position[1] << " " << data.bodyStateEst.position[2] << endl; 
                    cout << "body vBody: " << endl;
                    cout << data.bodyStateEst.vBody[0] << " " << data.bodyStateEst.vBody[1] << " " << data.bodyStateEst.vBody[2] << endl; 
                    cout << "pBody_des: " << endl;
                    cout << _wbc_data.pBody_des[0] << " " << _wbc_data.pBody_des[1] << " " << _wbc_data.pBody_des[2] << endl;
                    cout << "vBody_des: " << endl;
                    cout << _wbc_data.vBody_des[0] << " " << _wbc_data.vBody_des[1] << " " << _wbc_data.vBody_des[2] << endl;
                    cout << "aBody_des: " << endl;
                    cout << _wbc_data.aBody_des[0] << " " << _wbc_data.aBody_des[1] << " " << _wbc_data.aBody_des[2] << endl;

                    cout << "body rpy: " << endl;
                    cout << data.bodyStateEst.rpy[0] << " " << data.bodyStateEst.rpy[1] << " " << data.bodyStateEst.rpy[2] << endl; 
                    cout << "body omegaBody: " << endl;
                    cout << data.bodyStateEst.omegaBody[0] << " " << data.bodyStateEst.omegaBody[1] << " " << data.bodyStateEst.omegaBody[2] << endl; 
                    cout << "pBody_RPY_des: " << endl;
                    cout << _wbc_data.pBody_RPY_des[0] << " " << _wbc_data.pBody_RPY_des[1] << " " << _wbc_data.pBody_RPY_des[2] << endl;
                    cout << "pBody_RPY_now: " << endl;
                    cout << com_roll_now << " " << com_pitch_now << " " << com_yaw_now << endl;
                    cout << "vBody_RPY_des: " << endl;
                    cout << _wbc_data.vBody_RPY_des[0] << " " << _wbc_data.vBody_RPY_des[1] << " " << _wbc_data.vBody_RPY_des[2] << endl;
                    cout << "pfoot_des: " << endl;
                    cout << _wbc_data.pFoot_des[0] << "\n____\n" << _wbc_data.pFoot_des[1] << "\n____\n" << _wbc_data.pFoot_des[2] << "\n____\n" << _wbc_data.pFoot_des[3] << endl;

                    q.head(3) << data.bodyStateEst.position;
                    q.segment(3, 3) << data.bodyStateEst.rpy;
                    for(uint i = 0; i < 4; i++){
                        q.segment(6 + 3*i, 3) = data.legStateEst[i].q;
                    }
                    // cout << "q:______\n" << q;

                    // DVec<double> M_col_1 = 

                    v.head(3) = data.bodyStateEst.vBody;
                    v.segment(3, 3) =  data.bodyStateEst.omegaBody;
                    for(uint i = 0; i < 4; i++){
                        v.segment(6 + 3*i, 3) = data.legStateEst[i].qd;
                    }

                    in_x << _wbc_data.pBody_des[0] <<"\t" << data.bodyStateEst.position[0] << "\n";
                    in_y << _wbc_data.pBody_des[1] <<"\t" << data.bodyStateEst.position[1] << "\n";
                    in_z << _wbc_data.pBody_des[2] <<"\t" << data.bodyStateEst.position[2] << "\n";
                    in_roll << _wbc_data.pBody_RPY_des[0] <<"\t" << data.bodyStateEst.rpy[0] << "\n";
                    in_pitch << _wbc_data.pBody_RPY_des[1] <<"\t" << data.bodyStateEst.rpy[1] << "\n";
                    in_yaw << _wbc_data.pBody_RPY_des[2] <<"\t" << data.bodyStateEst.rpy[2] << "\n";

                    in_x_vel << _wbc_data.vBody_des[0] <<"\t" << data.bodyStateEst.vBody[0] << "\n";
                    in_y_vel << _wbc_data.vBody_des[1] <<"\t" << data.bodyStateEst.vBody[1] << "\n";
                    in_z_vel << _wbc_data.vBody_des[2] <<"\t" << data.bodyStateEst.vBody[2] << "\n";
                    in_roll_vel << _wbc_data.vBody_RPY_des[0] <<"\t" << data.bodyStateEst.omegaBody[0] << "\n";
                    in_pitch_vel << _wbc_data.vBody_RPY_des[1] <<"\t" << data.bodyStateEst.omegaBody[1] << "\n";
                    in_yaw_vel << _wbc_data.vBody_RPY_des[2] <<"\t" << data.bodyStateEst.omegaBody[2] << "\n";

                    DVec<float> foot_lf, foot_lh, foot_rf, foot_rh;
                    foot_lf = jueying.swingFootPosition(legID::LF, q.cast<double>()).cast<float>();
                    foot_lh = jueying.swingFootPosition(legID::LB, q.cast<double>()).cast<float>();
                    foot_rf = jueying.swingFootPosition(legID::RF, q.cast<double>()).cast<float>();
                    foot_rh = jueying.swingFootPosition(legID::RB, q.cast<double>()).cast<float>();

                    in_foot_lf_x << _wbc_data.pFoot_des[legID::LF][0] << "\t" << foot_lf[0] << "\n";
                    in_foot_lh_x << _wbc_data.pFoot_des[legID::LB][0] << "\t" << foot_lh[0] << "\n";
                    in_foot_rf_x << _wbc_data.pFoot_des[legID::RF][0] << "\t" << foot_rf[0] << "\n";
                    in_foot_rh_x << _wbc_data.pFoot_des[legID::RB][0] << "\t" << foot_rh[0] << "\n";

                    in_foot_lf_y << _wbc_data.pFoot_des[legID::LF][1] << "\t" << foot_lf[1] << "\n";
                    in_foot_lh_y << _wbc_data.pFoot_des[legID::LB][1] << "\t" << foot_lh[1] << "\n";
                    in_foot_rf_y << _wbc_data.pFoot_des[legID::RF][1] << "\t" << foot_rf[1] << "\n";
                    in_foot_rh_y << _wbc_data.pFoot_des[legID::RB][1] << "\t" << foot_rh[1] << "\n";

                    in_foot_lf_z << _wbc_data.pFoot_des[legID::LF][2] << "\t" << foot_lf[2] << "\n";
                    in_foot_lh_z << _wbc_data.pFoot_des[legID::LB][2] << "\t" << foot_lh[2] << "\n";
                    in_foot_rf_z << _wbc_data.pFoot_des[legID::RF][2] << "\t" << foot_rf[2] << "\n";
                    in_foot_rh_z << _wbc_data.pFoot_des[legID::RB][2] << "\t" << foot_rh[2] << "\n";

                    DVec<float> foot_lf_vel, foot_lh_vel, foot_rf_vel, foot_rh_vel;
                    foot_lf_vel = jueying.swingFootVelocity(legID::LF).cast<float>();
                    foot_lh_vel = jueying.swingFootVelocity(legID::LB).cast<float>();
                    foot_rf_vel = jueying.swingFootVelocity(legID::RF).cast<float>();
                    foot_rh_vel = jueying.swingFootVelocity(legID::RB).cast<float>();

                    in_foot_lf_x_vel << _wbc_data.vFoot_des[legID::LF][0] << "\t" << foot_lf_vel[0] << "\n";
                    in_foot_lh_x_vel << _wbc_data.vFoot_des[legID::LB][0] << "\t" << foot_lh_vel[0] << "\n";
                    in_foot_rf_x_vel << _wbc_data.vFoot_des[legID::RF][0] << "\t" << foot_rf_vel[0] << "\n";
                    in_foot_rh_x_vel << _wbc_data.vFoot_des[legID::RB][0] << "\t" << foot_rh_vel[0] << "\n";

                    in_foot_lf_y_vel << _wbc_data.vFoot_des[legID::LF][1] << "\t" << foot_lf_vel[1] << "\n";
                    in_foot_lh_y_vel << _wbc_data.vFoot_des[legID::LB][1] << "\t" << foot_lh_vel[1] << "\n";
                    in_foot_rf_y_vel << _wbc_data.vFoot_des[legID::RF][1] << "\t" << foot_rf_vel[1] << "\n";
                    in_foot_rh_y_vel << _wbc_data.vFoot_des[legID::RB][1] << "\t" << foot_rh_vel[1] << "\n";

                    in_foot_lf_z_vel << _wbc_data.vFoot_des[legID::LF][2] << "\t" << foot_lf_vel[2] << "\n";
                    in_foot_lh_z_vel << _wbc_data.vFoot_des[legID::LB][2] << "\t" << foot_lh_vel[2] << "\n";
                    in_foot_rf_z_vel << _wbc_data.vFoot_des[legID::RF][2] << "\t" << foot_rf_vel[2] << "\n";
                    in_foot_rh_z_vel << _wbc_data.vFoot_des[legID::RB][2] << "\t" << foot_rh_vel[2] << "\n";

                    in_vel_lf_z << _wbc_data.vFoot_des[legID::LF][2] << "\t" <<jueying.swingFootVelocity(legID::LF)[2] << "\n";
                    in_acc_lf_z << _wbc_data.aFoot_des[legID::LF][2] << "\t" << "0" << "\n";

                    in_contact_mpc << int(_wbc_data.contact_state[0]) << "\t"
                                    << int(_wbc_data.contact_state[1]) << "\t"
                                    << int(_wbc_data.contact_state[2]) << "\t"
                                    << int(_wbc_data.contact_state[3]) << "\n";
                                                        
                }

                iter_com++;
            }
            else{ // PD
                DataGlobal global_buffer;
                DataGlobal *global = &global_buffer;
                OneTypeLegData fl_final_angle, fr_final_angle, hl_final_angle, hr_final_angle;
                Vec31<double> fl_0, fl_1, fl_2;
                Vec31<double> hl_0, hl_1, hl_2;
                Vec31<double> fr_0, fr_1, fr_2;
                Vec31<double> hr_0, hr_1, hr_2;

                if(pd_0){
                    if((iter==0)&&(init_angle = true)){
                        for(int i(0); i<3; i++){
                            global->fl_initial_angle.value[i] = data_leg_now.fl_pos.value[i];
                            global->fr_initial_angle.value[i] = data_leg_now.fr_pos.value[i];
                            global->hl_initial_angle.value[i] = data_leg_now.hl_pos.value[i];
                            global->hr_initial_angle.value[i] = data_leg_now.hr_pos.value[i];
                        }
                        init_angle = false;
                    }
                    //JYPro Target Angle
                    fl_final_angle.value[0] = 0., fl_final_angle.value[1] = -1.23, fl_final_angle.value[2] = 2.79;
                    fr_final_angle.value[0] = 0., fr_final_angle.value[1] = -1.23, fr_final_angle.value[2] = 2.79;
                    hl_final_angle.value[0] = 0., hl_final_angle.value[1] = -1.23, hl_final_angle.value[2] = 2.79;
                    hr_final_angle.value[0] = 0., hr_final_angle.value[1] = -1.23, hr_final_angle.value[2] = 2.79;

                    double motion_time = 0.04;  // Planning time
                    double time_stamp = iter * cycle_time; // time now
                    if(iter==1){
                        std::cout << "initial angle fl: " << std::endl;
                        std::cout << global->fl_initial_angle.value[0] << " " << global->fl_initial_angle.value[1] << " " << global->fl_initial_angle.value[2] << std::endl;
                        std::cout << "initial angle hl: " << std::endl;
                        std::cout << global->hl_initial_angle.value[0] << " " << global->hl_initial_angle.value[1] << " " << global->hl_initial_angle.value[2] << std::endl;
                        std::cout << "initial angle fr: " << std::endl;
                        std::cout << global->fr_initial_angle.value[0] << " " << global->fr_initial_angle.value[1] << " " << global->fr_initial_angle.value[2] << std::endl;
                        std::cout << "initial angle hr: " << std::endl;
                        std::cout << global->hr_initial_angle.value[0] << " " << global->hr_initial_angle.value[1] << " " << global->hr_initial_angle.value[2] << std::endl;
                        std::cout << "\n" << std::endl;

                        std::cout << "final angle fl: " << std::endl;
                        std::cout << fl_final_angle.value[0] << " " << fl_final_angle.value[1] << " " << fl_final_angle.value[2] << std::endl;
                        std::cout << "final angle hl: " << std::endl;
                        std::cout << hl_final_angle.value[0] << " " << hl_final_angle.value[1] << " " << hl_final_angle.value[2] << std::endl;
                        std::cout << "final angle fr: " << std::endl;
                        std::cout << fr_final_angle.value[0] << " " << fr_final_angle.value[1] << " " << fr_final_angle.value[2] << std::endl;
                        std::cout << "final angle hr: " << std::endl;
                        std::cout << hr_final_angle.value[0] << " " << hr_final_angle.value[1] << " " << hr_final_angle.value[2] << std::endl;

                    }

                    if (time_stamp <= motion_time)
                    //if(false)
                    {
                        fl_0 = TrajectoryPlan_d(global->fl_initial_angle.value[0], fl_final_angle.value[0], motion_time, time_stamp);
                        fl_1 = TrajectoryPlan_d(global->fl_initial_angle.value[1], fl_final_angle.value[1], motion_time, time_stamp);
                        fl_2 = TrajectoryPlan_d(global->fl_initial_angle.value[2], fl_final_angle.value[2], motion_time, time_stamp);

                        hl_0 = TrajectoryPlan_d(global->hl_initial_angle.value[0], hl_final_angle.value[0], motion_time, time_stamp);
                        hl_1 = TrajectoryPlan_d(global->hl_initial_angle.value[1], hl_final_angle.value[1], motion_time, time_stamp);
                        hl_2 = TrajectoryPlan_d(global->hl_initial_angle.value[2], hl_final_angle.value[2], motion_time, time_stamp);

                        fr_0 = TrajectoryPlan_d(global->fr_initial_angle.value[0], fr_final_angle.value[0], motion_time, time_stamp);
                        fr_1 = TrajectoryPlan_d(global->fr_initial_angle.value[1], fr_final_angle.value[1], motion_time, time_stamp);
                        fr_2 = TrajectoryPlan_d(global->fr_initial_angle.value[2], fr_final_angle.value[2], motion_time, time_stamp);

                        hr_0 = TrajectoryPlan_d(global->hr_initial_angle.value[0], hr_final_angle.value[0], motion_time, time_stamp);
                        hr_1 = TrajectoryPlan_d(global->hr_initial_angle.value[1], hr_final_angle.value[1], motion_time, time_stamp);
                        hr_2 = TrajectoryPlan_d(global->hr_initial_angle.value[2], hr_final_angle.value[2], motion_time, time_stamp);
                    }

                    else{
                        pd_1= true;
                        init_angle = true;
                        pd_0 = false;
                    }
                    
                    iter++;
                }


                if(pd_1){
                    if((iter_1==0)&&(init_angle = true)){
                        for(int i(0); i<3; i++){
                            global->fl_initial_angle.value[i] = data_leg_now.fl_pos.value[i];
                            global->fr_initial_angle.value[i] = data_leg_now.fr_pos.value[i];
                            global->hl_initial_angle.value[i] = data_leg_now.hl_pos.value[i];
                            global->hr_initial_angle.value[i] = data_leg_now.hr_pos.value[i];
                        }
                        init_angle = false;
                    }
                    //JYPro Target Angle
                    fl_final_angle.value[0] = 0., fl_final_angle.value[1] = -0.95, fl_final_angle.value[2] = 1.7;
                    fr_final_angle.value[0] = 0., fr_final_angle.value[1] = -0.95, fr_final_angle.value[2] = 1.7;
                    hl_final_angle.value[0] = 0., hl_final_angle.value[1] = -0.95, hl_final_angle.value[2] = 1.7;
                    hr_final_angle.value[0] = 0., hr_final_angle.value[1] = -0.95, hr_final_angle.value[2] = 1.7;

                    double motion_time = 1;  // Planning time
                    double time_stamp = iter_1 * cycle_time; // time now
                    if(iter_1==1){
                        std::cout << "initial angle fl: " << std::endl;
                        std::cout << global->fl_initial_angle.value[0] << " " << global->fl_initial_angle.value[1] << " " << global->fl_initial_angle.value[2] << std::endl;
                        std::cout << "initial angle hl: " << std::endl;
                        std::cout << global->hl_initial_angle.value[0] << " " << global->hl_initial_angle.value[1] << " " << global->hl_initial_angle.value[2] << std::endl;
                        std::cout << "initial angle fr: " << std::endl;
                        std::cout << global->fr_initial_angle.value[0] << " " << global->fr_initial_angle.value[1] << " " << global->fr_initial_angle.value[2] << std::endl;
                        std::cout << "initial angle hr: " << std::endl;
                        std::cout << global->hr_initial_angle.value[0] << " " << global->hr_initial_angle.value[1] << " " << global->hr_initial_angle.value[2] << std::endl;
                        std::cout << "\n" << std::endl;

                        std::cout << "final angle fl: " << std::endl;
                        std::cout << fl_final_angle.value[0] << " " << fl_final_angle.value[1] << " " << fl_final_angle.value[2] << std::endl;
                        std::cout << "final angle hl: " << std::endl;
                        std::cout << hl_final_angle.value[0] << " " << hl_final_angle.value[1] << " " << hl_final_angle.value[2] << std::endl;
                        std::cout << "final angle fr: " << std::endl;
                        std::cout << fr_final_angle.value[0] << " " << fr_final_angle.value[1] << " " << fr_final_angle.value[2] << std::endl;
                        std::cout << "final angle hr: " << std::endl;
                        std::cout << hr_final_angle.value[0] << " " << hr_final_angle.value[1] << " " << hr_final_angle.value[2] << std::endl;

                    }

                    if (time_stamp <= motion_time)
                    {
                        fl_0 = TrajectoryPlan_d(global->fl_initial_angle.value[0], fl_final_angle.value[0], motion_time, time_stamp);
                        fl_1 = TrajectoryPlan_d(global->fl_initial_angle.value[1], fl_final_angle.value[1], motion_time, time_stamp);
                        fl_2 = TrajectoryPlan_d(global->fl_initial_angle.value[2], fl_final_angle.value[2], motion_time, time_stamp);

                        hl_0 = TrajectoryPlan_d(global->hl_initial_angle.value[0], hl_final_angle.value[0], motion_time, time_stamp);
                        hl_1 = TrajectoryPlan_d(global->hl_initial_angle.value[1], hl_final_angle.value[1], motion_time, time_stamp);
                        hl_2 = TrajectoryPlan_d(global->hl_initial_angle.value[2], hl_final_angle.value[2], motion_time, time_stamp);

                        fr_0 = TrajectoryPlan_d(global->fr_initial_angle.value[0], fr_final_angle.value[0], motion_time, time_stamp);
                        fr_1 = TrajectoryPlan_d(global->fr_initial_angle.value[1], fr_final_angle.value[1], motion_time, time_stamp);
                        fr_2 = TrajectoryPlan_d(global->fr_initial_angle.value[2], fr_final_angle.value[2], motion_time, time_stamp);

                        hr_0 = TrajectoryPlan_d(global->hr_initial_angle.value[0], hr_final_angle.value[0], motion_time, time_stamp);
                        hr_1 = TrajectoryPlan_d(global->hr_initial_angle.value[1], hr_final_angle.value[1], motion_time, time_stamp);
                        hr_2 = TrajectoryPlan_d(global->hr_initial_angle.value[2], hr_final_angle.value[2], motion_time, time_stamp);
                    }
                    else{
                        fl_0 = TrajectoryPlan_d(fl_final_angle.value[0], fl_final_angle.value[0], motion_time, time_stamp);
                        fl_1 = TrajectoryPlan_d(fl_final_angle.value[1], fl_final_angle.value[1], motion_time, time_stamp);
                        fl_2 = TrajectoryPlan_d(fl_final_angle.value[2], fl_final_angle.value[2], motion_time, time_stamp);

                        hl_0 = TrajectoryPlan_d(hl_final_angle.value[0], hl_final_angle.value[0], motion_time, time_stamp);
                        hl_1 = TrajectoryPlan_d(hl_final_angle.value[1], hl_final_angle.value[1], motion_time, time_stamp);
                        hl_2 = TrajectoryPlan_d(hl_final_angle.value[2], hl_final_angle.value[2], motion_time, time_stamp);

                        fr_0 = TrajectoryPlan_d(fr_final_angle.value[0], fr_final_angle.value[0], motion_time, time_stamp);
                        fr_1 = TrajectoryPlan_d(fr_final_angle.value[1], fr_final_angle.value[1], motion_time, time_stamp);
                        fr_2 = TrajectoryPlan_d(fr_final_angle.value[2], fr_final_angle.value[2], motion_time, time_stamp);

                        hr_0 = TrajectoryPlan_d(hr_final_angle.value[0], hr_final_angle.value[0], motion_time, time_stamp);
                        hr_1 = TrajectoryPlan_d(hr_final_angle.value[1], hr_final_angle.value[1], motion_time, time_stamp);
                        hr_2 = TrajectoryPlan_d(hr_final_angle.value[2], hr_final_angle.value[2], motion_time, time_stamp);


                        ++iterpdDone;
                        if(iterpdDone > 1000)
                            pdDone = true;
                    }

                    iter_1++;
                }

                tau_lf_haa.data = 800 * (fl_0[0] - data_leg_now.fl_pos.value[0]) + 10 * (fl_0[1] - data_leg_now.fl_vel.value[0]);//15
                tau_lf_hfe.data = 800 * (fl_1[0] - data_leg_now.fl_pos.value[1]) + 10 * (fl_1[1] - data_leg_now.fl_vel.value[1]);//15
                tau_lf_kfe.data = 800 * (fl_2[0] - data_leg_now.fl_pos.value[2]) + 10 * (fl_2[1] - data_leg_now.fl_vel.value[2]);//20

                tau_lb_haa.data = 800 * (hl_0[0] - data_leg_now.hl_pos.value[0]) + 10 * (hl_0[1] - data_leg_now.hl_vel.value[0]);//17
                tau_lb_hfe.data = 800 * (hl_1[0] - data_leg_now.hl_pos.value[1]) + 10 * (hl_1[1] - data_leg_now.hl_vel.value[1]);//17
                tau_lb_kfe.data = 800 * (hl_2[0] - data_leg_now.hl_pos.value[2]) + 10 * (hl_2[1] - data_leg_now.hl_vel.value[2]);//22

                tau_rf_haa.data = 800 * (fr_0[0] - data_leg_now.fr_pos.value[0]) + 10 * (fr_0[1] - data_leg_now.fr_vel.value[0]);//15
                tau_rf_hfe.data = 800 * (fr_1[0] - data_leg_now.fr_pos.value[1]) + 10 * (fr_1[1] - data_leg_now.fr_vel.value[1]);//15
                tau_rf_kfe.data = 800 * (fr_2[0] - data_leg_now.fr_pos.value[2]) + 10 * (fr_2[1] - data_leg_now.fr_vel.value[2]);//20    

                tau_rb_haa.data = 800 * (hr_0[0] - data_leg_now.hr_pos.value[0]) + 10 * (hr_0[1] - data_leg_now.hr_vel.value[0]);//17
                tau_rb_hfe.data = 800 * (hr_1[0] - data_leg_now.hr_pos.value[1]) + 10 * (hr_1[1] - data_leg_now.hr_vel.value[1]);//17
                tau_rb_kfe.data = 800 * (hr_2[0] - data_leg_now.hr_pos.value[2]) + 10 * (hr_2[1] - data_leg_now.hr_vel.value[2]);//22


            }
            lf_haa.publish(tau_lf_haa); lf_hfe.publish(tau_lf_hfe); lf_kfe.publish(tau_lf_kfe);
            lb_haa.publish(tau_lb_haa); lb_hfe.publish(tau_lb_hfe); lb_kfe.publish(tau_lb_kfe);
            rf_haa.publish(tau_rf_haa); rf_hfe.publish(tau_rf_hfe); rf_kfe.publish(tau_rf_kfe);
            rb_haa.publish(tau_rb_haa); rb_hfe.publish(tau_rb_hfe); rb_kfe.publish(tau_rb_kfe);
            planner_on.publish(if_plan);
        }
        ros::spinOnce();
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
        // spinner.stop();
        bool rate_bool = rate.sleep();
        // std::cerr << "####sleep:" <<  rate_bool << "\n";
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::cerr << "time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;
    }

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
    in_foot_lh_x.close();
    in_foot_rf_x.close();
    in_foot_rh_x.close();

    in_foot_lf_y.close();
    in_foot_lh_y.close();
    in_foot_rf_y.close();
    in_foot_rh_y.close();

    in_foot_lf_z.close();
    in_foot_lh_z.close();
    in_foot_rf_z.close();
    in_foot_rh_z.close();

    in_foot_lf_x_vel.close();
    in_foot_lh_x_vel.close();
    in_foot_rf_x_vel.close();
    in_foot_rh_x_vel.close();

    in_foot_lf_y_vel.close();
    in_foot_lh_y_vel.close();
    in_foot_rf_y_vel.close();
    in_foot_rh_y_vel.close();

    in_foot_lf_z_vel.close();
    in_foot_lh_z_vel.close();
    in_foot_rf_z_vel.close();
    in_foot_rh_z_vel.close();

    in_vel_lf_z.close();
    in_acc_lf_z.close();

    in_contact_mpc.close();


    return 0;
}