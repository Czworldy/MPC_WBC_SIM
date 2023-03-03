#pragma GCC optimize(2)
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include "ros/ros.h"
#include "time.h"
#include <ros/node_handle.h>
#include <ros/package.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "pronto_msgs/QuadrupedStance.h"
// #include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"

#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_state.h>
#include <ocs2_msgs/mpc_input.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_msgs/mpc_observation.h>
#include "ocs2_jypro/gait/MotionPhaseDefinition.h"
#include <gazebo_msgs/ContactsState.h>

#include "BodyPositionEstimator.h"

using namespace ocs2;

struct MPCInputData {
    scalar_t time_;
    vector_t q_; // Generalized Coordinates (18 for quadruped)
    vector_t v_; // Generalized Velocities (18 for quadruped)
    vector_t grf_; // 12 for quadruped ---- LF RF LH RH
    vector_t stance_; // 4 for quadruped ---- LF LH RF RH (For Body Position Estimator)
    legged_robot::contact_flag_t stance_bool_; // 4 for quadruped ---- LF RF LH RH (For MPC Input)
    int numOfStance_;
};

// Global Variables
MPCInputData mpcInputData;
uint numOfActuatedJoint(12);
uint numOfContactPoint(4);
uint dofOfRobot(18);
bool debug(true);

bool jointStateMsg(false);
bool gazeboStates_Msg(false);
bool contactMsg(false);
bool grfLFMsg(false);
bool grfRFMsg(false);
bool grfLHMsg(false);
bool grfRHMsg(false);
int  prontoCallbackData_count = 0;
int  prontoData_count = 0;


// Functions
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void gazebo_link_states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
void contactStateCallback(const pronto_msgs::QuadrupedStance::ConstPtr& msg);
void grf_LF_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void grf_RF_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void grf_LH_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void grf_RH_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

Eigen::Matrix<double, 3, 1> quaternionTOrpy(Eigen::Quaternion<double> q);
Eigen::Matrix<double, 3, 3> rpyTORotateMat(double roll, double pitch, double yaw);
Eigen::Matrix<double, 3, 3> rpyDotTOtwist(double yaw, double pitch, double roll);

// Main
int main(int argc, char **argv)
{   
    // ROS_Related Variables
    ros::init(argc, argv, "MPCProntoConversion");
    ros::NodeHandle nh;
    ros::Subscriber jointState, contactState;
    ros::Subscriber gazebo_linkStates;
    ros::Subscriber grf_LF, grf_RF, grf_LH, grf_RH;
    ros::Publisher mpc_input;
    ocs2_msgs::mpc_observation mpc_input_msg;
    ros::Rate rate(200);
    // Main Variables
    // BodyPositionEst bodyPositionEst;
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
    legged_robot::ModelSettings modelSettings;

    jointState = nh.subscribe("/X20/joint_states", 1, &jointStateCallback);
    contactState = nh.subscribe("/state_estimator_pronto/stance", 1, &contactStateCallback);
    gazebo_linkStates = nh.subscribe("/gazebo/model_states", 1,&gazebo_link_states_callback);
    mpc_input = nh.advertise<ocs2_msgs::mpc_observation>("/legged_robot_mpc_observation", 1);
    grf_LF = nh.subscribe("/state_estimator_pronto/lf_grf", 1, &grf_LF_callback);
    grf_RF = nh.subscribe("/state_estimator_pronto/rf_grf", 1, &grf_RF_callback);
    grf_LH = nh.subscribe("/state_estimator_pronto/lh_grf", 1, &grf_LH_callback);
    grf_RH = nh.subscribe("/state_estimator_pronto/rh_grf", 1, &grf_RH_callback);

    mpcInputData.q_.resize(dofOfRobot);
    mpcInputData.v_.resize(dofOfRobot);
    mpcInputData.stance_.resize(numOfContactPoint);
    mpcInputData.grf_.resize(3 * numOfContactPoint);

    // URDF Model -> Pinocchio Model
    std::string urdfFilePath;
    nh.getParam("/urdfFile", urdfFilePath);
    // if (!ros::param::get("/legged_robot_description", urdfFilePath)) {
    //   std::cerr << "Param " << "/legged_robot_description" << " not found; unable to generate urdf" << std::endl;
    // }
    pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFilePath, modelSettings.jointNames)));

    while(nh.ok()){
        if(jointStateMsg && contactMsg && grfLFMsg && grfRFMsg && grfLHMsg && grfRHMsg && gazeboStates_Msg){

            mpc_input_msg.state.value.resize(6 + dofOfRobot);
            const auto& model = pinocchioInterfacePtr->getModel();
            auto& data = pinocchioInterfacePtr->getData();
            pinocchio::forwardKinematics(model, data, mpcInputData.q_, mpcInputData.v_);
            
            const auto& Ag = pinocchio::computeCentroidalMap(model, data, mpcInputData.q_);
            const auto& Hcom = Ag * mpcInputData.v_;
            pinocchio::computeTotalMass(model, data);
            // Centroidal Momtentum
            for(uint i = 0; i < 3; i++){
                mpc_input_msg.state.value[i] = Hcom[i] / data.mass[0]; 
                mpc_input_msg.state.value[i + 3] = Hcom[i+3] / data.mass[0];  
            }
            // XYZ RPY
            for(uint i = 0; i < 6; i++){
                mpc_input_msg.state.value[i + 6] = mpcInputData.q_[i];
            }
            // mpc_input_msg.state.value[6] = 0.;
            // mpc_input_msg.state.value[7] = 0.;
            // mpc_input_msg.state.value[8] = 0.;
            // mpc_input_msg.state.value[9] = 0.;

            // Actuated joint
            for(uint i = 0; i < numOfActuatedJoint; i++){
                mpc_input_msg.state.value[i + 12] = mpcInputData.q_[i + 6];
            }

            // mpc_input_msg
            mpc_input_msg.input.value.resize(3 * numOfContactPoint + numOfActuatedJoint);
            // Contact Force 
            uint index = 0;
            for(uint i = 0; i < numOfContactPoint; i++){
                if(mpcInputData.stance_bool_[i]){
                    for(uint j = 0; j < 3; j++){
                        mpc_input_msg.input.value[index + j] = mpcInputData.grf_[3 * i + j];
                    }
                    index += 3;
                }
                else {
                    for(uint j = 0; j < 3; j++){
                        mpc_input_msg.input.value[index + j] = 0;
                    }
                    index += 3;
                }
            }
            // Actuated joint
            for(uint i = 0; i < numOfActuatedJoint; i++){
                mpc_input_msg.input.value[3 * numOfContactPoint + i] = mpcInputData.v_[6 + i];
            }

            // mpc_time_msg 
            mpc_input_msg.time = ros::Time::now().toSec();
            // mpc_mode_msg
            mpc_input_msg.mode = int(legged_robot::stanceLeg2ModeNumber(mpcInputData.stance_bool_));

            mpc_input.publish(mpc_input_msg);

            if(debug){
                std::cout << "publish msg done!" << std::endl;
                std::cout << "____________MPC Input Massage:____MPC_PRONTO_CONVERSION_________" << std::endl;
                std::cout << "MPC State:____________ " << std::endl;
                std::cout << "Centrodial Momentum: x y z roll pitch yaw" <<std::endl;
                for(uint i = 0; i < 6; i++){
                    std::cout << mpc_input_msg.state.value[i] << std::endl;
                }
                std::cout << "Body Pose: x y z yaw pitch roll" << std::endl;
                for(uint i = 0; i < 6; i++){
                    std::cout << mpc_input_msg.state.value[i + 6] << std::endl;
                }
                std::cout << "Actuated Joints:" << std::endl;
                for(uint i = 0; i < numOfActuatedJoint; i++){
                    std::cout << mpc_input_msg.state.value[i + 12] << std::endl;
                }
                std::cout << "MPC Input:___________ " << std::endl;
                std::cout << "Contact Point Forces: " << std::endl;
                for(uint i = 0; i < mpc_input_msg.input.value.size() - numOfActuatedJoint; i++){
                    std::cout << mpc_input_msg.input.value[i] << std::endl;
                }
                std::cout << "Actuated Joints: " << std::endl;  
                for(uint i = 0; i < numOfActuatedJoint; i++){
                    std::cout << mpc_input_msg.input.value[mpc_input_msg.input.value.size() - 12 + i] << std::endl;
                }
                std::cout << "Gait Mode:____________" << std::endl;
                std::cout << mpcInputData.stance_bool_[0] << std::endl;
                std::cout << mpcInputData.stance_bool_[1] << std::endl;
                std::cout << mpcInputData.stance_bool_[2] << std::endl;
                std::cout << mpcInputData.stance_bool_[3] << std::endl;
                std::cout << double(mpc_input_msg.mode) << std::endl;
                std::cout << "time:____________" << std::endl;
                std::cout << mpc_input_msg.time << std::endl;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    //  LF_HAA, LF_HFE, LF_KFE
    for(uint i(0); i<3; i++){
        mpcInputData.q_[i + 6] = msg->position[i];
        mpcInputData.v_[i + 6] = msg->velocity[i];
    }

    //  RF_HAA, RF_HFE, RF_KFE
    for(uint i(0); i<3; i++){
        mpcInputData.q_[i + 12] = msg->position[i + 3];
        mpcInputData.v_[i + 12] = msg->velocity[i + 3];
    }

    //  LH_HAA, LH_HFE, LH_KFE
    for(uint i(0); i<3; i++){
        mpcInputData.q_[i + 9] = msg->position[i + 6];
        mpcInputData.v_[i + 9] = msg->velocity[i + 6];
    }   

    //  RH_HAA, RH_HFE, RH_KFE
    for(uint i(0); i<3; i++){
        mpcInputData.q_[i + 15] = msg->position[i + 9];
        mpcInputData.v_[i + 15] = msg->velocity[i + 9];
    }

    jointStateMsg = true;
}

void contactStateCallback(const pronto_msgs::QuadrupedStance::ConstPtr& msg){
    mpcInputData.numOfStance_ = 0;
    // LF
    mpcInputData.stance_[0] = msg->lf; // For Body Position Estimator ---- LF LH RF RH
    mpcInputData.stance_bool_[0] = msg->lf; // For MPC Input Data ---- LF RF LH RH
    // RF
    mpcInputData.stance_[1] = msg->lh; 
    mpcInputData.stance_bool_[1] = msg->rf;
    // LH
    mpcInputData.stance_[2] = msg->rf;
    mpcInputData.stance_bool_[2] = msg->lh;
    // RH
    mpcInputData.stance_[3] = msg->rh;
    mpcInputData.stance_bool_[3] = msg->rh;

    for(uint i = 0; i < numOfContactPoint; i++){
        if(mpcInputData.stance_[i]){
            mpcInputData.numOfStance_++;
        }
    }

    contactMsg = true;
}


QuaternionToRPY yawTotalCounter;

void gazebo_link_states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    // pose --- orientation
    // double ros_time = ros::Time::now().toSec();
    Eigen::Quaternion<double> orientation_gazebo, orientation_tpl, orientation_final;
    Eigen::Matrix<double,3,1> rpy;
    int index = 0;
    for(auto& modelName:msg->name){
        if(modelName == "X20")
            break;
        ++index;
    }

    
    orientation_gazebo.w() = msg->pose[index].orientation.w;
    orientation_gazebo.x() = msg->pose[index].orientation.x;
    orientation_gazebo.y() = msg->pose[index].orientation.y;
    orientation_gazebo.z() = msg->pose[index].orientation.z;

    rpy = yawTotalCounter.quaternionToTotalRad(orientation_gazebo);
    
    // Yaw
    mpcInputData.q_[3] = rpy[2];
    // Pitch
    mpcInputData.q_[4] = rpy[1];
    // Roll
    mpcInputData.q_[5] = rpy[0];

    // pose --- position
    Eigen::Matrix<double,3,1> position;
    position[0] = msg->pose[index].position.x; 
    position[1] = msg->pose[index].position.y;  
    position[2] = msg->pose[index].position.z;
    mpcInputData.q_.head(3) =   position;
    
    // twist --- angular
    Eigen::Matrix<double,3,1> twist_angular, twist_angular_tpl;
    twist_angular[0] = msg->twist[index].angular.x;
    twist_angular[1] = msg->twist[index].angular.y;
    twist_angular[2] = msg->twist[index].angular.z;
    twist_angular_tpl =   twist_angular;
    mpcInputData.v_[3] = twist_angular_tpl[2];
    mpcInputData.v_[4] = twist_angular_tpl[1];
    mpcInputData.v_[5] = twist_angular_tpl[0];
    
    // twist --- linear
    Eigen::Matrix<double,3,1> twist_linear;
    twist_linear[0] = msg->twist[index].linear.x;
    twist_linear[1] = msg->twist[index].linear.y;
    twist_linear[2] = msg->twist[index].linear.z;
    mpcInputData.v_.head(3) =   twist_linear;

    gazeboStates_Msg = true; 

}

void grf_LF_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    Eigen::Matrix<double,3,1> grf, grf_tpl;
    grf[0] = msg->wrench.force.x;
    grf[1] = msg->wrench.force.y;
    grf[2] = msg->wrench.force.z;
    grf_tpl << 0,0,0;//=   grf;
    mpcInputData.grf_[0] = grf_tpl[0];
    mpcInputData.grf_[1] = grf_tpl[1];
    mpcInputData.grf_[2] = grf_tpl[2];

    grfLFMsg = true;
}

void grf_RF_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    Eigen::Matrix<double,3,1> grf, grf_tpl;
    grf[0] = msg->wrench.force.x;
    grf[1] = msg->wrench.force.y;
    grf[2] = msg->wrench.force.z;
    grf_tpl << 0,0,0;//=   grf;
    mpcInputData.grf_[3] = grf_tpl[0];
    mpcInputData.grf_[4] = grf_tpl[1];
    mpcInputData.grf_[5] = grf_tpl[2];

    grfRFMsg = true;
}

void grf_LH_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    Eigen::Matrix<double,3,1> grf, grf_tpl;
    grf[0] = msg->wrench.force.x;
    grf[1] = msg->wrench.force.y;
    grf[2] = msg->wrench.force.z;
    grf_tpl << 0,0,0;//=   grf;
    mpcInputData.grf_[6] = grf_tpl[0];
    mpcInputData.grf_[7] = grf_tpl[1];
    mpcInputData.grf_[8] = grf_tpl[2];

    grfLHMsg = true;
}

void grf_RH_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    Eigen::Matrix<double,3,1> grf, grf_tpl;
    grf[0] = msg->wrench.force.x;
    grf[1] = msg->wrench.force.y;
    grf[2] = msg->wrench.force.z;
    grf_tpl << 0,0,0;//=   grf;
    mpcInputData.grf_[9]  = grf_tpl[0];
    mpcInputData.grf_[10] = grf_tpl[1];
    mpcInputData.grf_[11] = grf_tpl[2];

    grfRHMsg = true;
}

Eigen::Matrix<double, 3, 1> quaternionTOrpy(Eigen::Quaternion<double> q){
    Eigen::Matrix<double, 3, 1> rpy;
    rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
    rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
    rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));
    // if(rpy[2] < -0.1)
    //     rpy[2] += 2*M_PI;
    return rpy;
 }


Eigen::Matrix<double, 3, 3> rpyTORotateMat(double roll, double pitch, double yaw){
    Eigen::Matrix<double, 3, 3> RotateMatrix, R_roll, R_pitch, R_yaw;
    R_roll <<  1., 0., 0., 
               0., cos(roll), -sin(roll),
               0., sin(roll), cos(roll);
    R_pitch << cos(pitch), 0, sin(pitch),
                0., 1., 0.,
              -sin(pitch), 0., cos(pitch);
    R_yaw << cos(yaw), -sin(yaw), 0.,
             sin(yaw), cos(yaw), 0.,
              0., 0., 1.;
    RotateMatrix = R_yaw * R_pitch * R_roll;
    return RotateMatrix;
}

Eigen::Matrix<double, 3, 3> rpyDotTOtwist(double yaw, double pitch, double roll){
    Eigen::Matrix<double, 3, 3> translation_Matrix;

    translation_Matrix << 0, -sin(yaw), cos(pitch) * cos(yaw),
                          0, cos(yaw), cos(pitch) * sin(yaw),
                          1, 0 , -sin(pitch);
                          
    return translation_Matrix;
}
