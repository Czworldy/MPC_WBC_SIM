#pragma GCC optimize(2)
// C++
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <iostream>
#include <eigen3/Eigen/Geometry>
// pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
// ros
#include "ros/ros.h"
#include "time.h"
#include <ros/node_handle.h>
#include <ros/package.h>
// ocs2
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_state.h>
#include <ocs2_msgs/mpc_input.h>
#include <ocs2_msgs/mpc_terrain.h>
#include <ocs2_msgs/reset.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_msgs/mpc_observation.h>
#include "ocs2_jypro/gait/MotionPhaseDefinition.h"
#include "ocs2_jypro/synchronized_module/TerrainReceiver.h"

#include <ocs2_core/misc/LoadData.h>
// MPC messages
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

#include "BodyPositionEstimator.h"
// UDP
#define SERVER_PORT 8888
#define SERVER_IP "127.0.0.1"
// ocs2
using namespace ocs2;

// Struct
struct LimbsContacts {
public:
    float lf;
    float rf;
    float lh;
    float rh;
};
typedef struct
{
	double value[3];
} OneLimbData;

typedef struct
{
	OneLimbData lf_pos;
	OneLimbData rf_pos;
	OneLimbData lh_pos;
	OneLimbData rh_pos;
	OneLimbData lf_vel;
	OneLimbData rf_vel;
	OneLimbData lh_vel;
	OneLimbData rh_vel;
}	LimbsPosVel;

// struct TerrainEstData{
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// public:
//     Eigen::Quaternionf terrainQuat;
//     Eigen::Vector3f terrainParams;
// };

struct EstimatorOutput {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    float time_stamp;
    Eigen::Matrix<double, 3, 1> base_pos_world;
    Eigen::Matrix<double, 3, 1> base_linear_vel_world;
    Eigen::Matrix<double, 3, 1> base_linear_vel_body;
    Eigen::Quaterniond base_orientation_world;
    Eigen::Matrix<double, 3, 1> base_angular_vel_world;
    Eigen::Matrix<double, 3, 1> base_angular_vel_body;
    LimbsContacts contact;
    LimbsPosVel jointStates;

    Eigen::Matrix<double, 3, 1> frame_c_rpy_in_world;
    Eigen::Quaterniond frame_c_quat_in_world;
    Eigen::Matrix<double, 3, 1> frame_c_xyz_in_world;

    std::vector<Eigen::Vector3d> foot_position;
    std::vector<Eigen::Vector3d> foot_position_for_terrain;
    ocs2::legged_robot::TerrainEstData terrainEstData;
#ifdef USE_TERRAIN
    std::vector<Eigen::Vector3d> foot_position;
    Eigen::Quaterniond terrain_orientation;
    Eigen::Vector3d terrain_params;
#endif
};

struct MPCInputData {
    scalar_t time_;
    vector_t q_; // Generalized Coordinates (18 for quadruped)
    vector_t v_; // Generalized Velocities (18 for quadruped)
    vector_t stance_; // 4 for quadruped ---- LF LH RF RH (For Body Position Estimator)
    legged_robot::contact_flag_t stance_bool_; // 4 for quadruped ---- LF RF LH RH (For MPC Input)
    int numOfStance_;
#ifdef USE_TERRAIN
    matrix_t terrainRotMat;
    Eigen::Vector3d terrain_params;
#endif
};

// Global Variables
MPCInputData mpcInputData;
MPCInputData mpcInputData_pre;
double z_ref_pre;
bool firstRun(true);
uint numOfActuatedJoint(12);
uint numOfContactPoint(4);
uint dofOfRobot(18);
bool debug(true);
Eigen::Quaternion<double> baseQuat;
Eigen::Matrix<double, 3, 1> baseRPY;
bool isReset(false);

// Function
using matrix3_t = Eigen::Matrix<double, 3, 3>;
using matrix3_t = Eigen::Matrix<double, 3, 3>;
Eigen::Matrix<double, 3, 1> quaternionTOrpy(Eigen::Quaternion<double> q);
matrix3_t rpyDotTOtwist(double theta_z, double theta_y, double theta_x);
QuaternionToRPY yawTotalCounter;
// Main
int main(int argc, char **argv) {
    //FOR UDP CLIENT
    int rec_fd;
    struct sockaddr_in rec_aadr;

    rec_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(rec_fd < 0){
        printf("create socket fail!\n");
        return -1;
    }

    memset(&rec_aadr, 0 , sizeof(rec_aadr));
    rec_aadr.sin_family = AF_INET;
    rec_aadr.sin_addr.s_addr = htonl(INADDR_ANY);
    rec_aadr.sin_port = htons(SERVER_PORT);

    EstimatorOutput buf;
    memset(&buf, 0, sizeof(EstimatorOutput));

    socklen_t len;
    size_t buf_len;

    len = sizeof(rec_aadr);
    buf_len = sizeof(buf);

    std::cout << buf_len << "\n";
    if (bind(rec_fd, (sockaddr *)&rec_aadr, sizeof(rec_aadr)) == -1) {
        std::cerr << ">>>>>>>>>>>>>>>>>>UDPInit:time bind failed:\n";
        return 3;
    }
    std::cout << "UDP init done!\n";
    // ROS
    ros::init(argc, argv, "MPCProntoConversionUDP");
    ros::NodeHandle nh;
    ros::Publisher mpc_input, mpc_terrain_sync_input;
    ros::ServiceClient mpcResetServiceClient_ = nh.serviceClient<ocs2_msgs::reset>("/legged_robot_mpc_reset");
    ocs2_msgs::mpc_observation mpc_input_msg;
    ocs2_msgs::mpc_terrain mpc_terrain_sync_input_msg;
    ros::Rate rate(200);

    mpc_input = nh.advertise<ocs2_msgs::mpc_observation>("/legged_robot_mpc_observation", 1);
    mpc_terrain_sync_input = nh.advertise<ocs2_msgs::mpc_terrain>("/legged_robot_mpc_terrain", 1);

    // Main Variables
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
    legged_robot::ModelSettings modelSettings;

    mpcInputData.q_.resize(dofOfRobot);
    mpcInputData.v_.resize(dofOfRobot);
    mpcInputData.stance_.resize(numOfContactPoint);

    // URDF Model -> Pinocchio Model
    std::string urdfFilePath;

    nh.getParam("/urdfFile", urdfFilePath);
    pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFilePath, modelSettings.jointNames)));

    // Target
    // vector_t defaultJointState(12);

    // const std::string targetCommandFile = "/home/nuc/MPC_WBC/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/targetTrajectories.info";
    // boost::property_tree::ptree pt;
    // boost::property_tree::read_info(targetCommandFile, pt);
    // ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState);
    std::cout << "################## Start recvfrom loops! ##################\n";

    while(nh.ok()){
        // Recieve Estimator Output Data*******************************************************
        recvfrom(rec_fd, &buf, buf_len, 0, (struct sockaddr*)&rec_aadr, &len);
        std::chrono::steady_clock::time_point recv_tp = std::chrono::steady_clock::now();
        // auto duration = recv_tp.time_since_epoch();
        std::cerr << "\nreceive time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(recv_tp.time_since_epoch()).count() << "\n";
        // Convert EstimatorOutput to MpcInput
        mpcInputData.time_ = buf.time_stamp;
        // JointStates
        for(uint i(0); i<3; i++) {
            // q
            mpcInputData.q_[i + 6]  = buf.jointStates.lf_pos.value[i];
            mpcInputData.q_[i + 9]  = buf.jointStates.lh_pos.value[i];
            mpcInputData.q_[i + 12] = buf.jointStates.rf_pos.value[i];
            mpcInputData.q_[i + 15] = buf.jointStates.rh_pos.value[i];
            // v
            mpcInputData.v_[i + 6]  = buf.jointStates.lf_vel.value[i];
            mpcInputData.v_[i + 9]  = buf.jointStates.lh_vel.value[i];
            mpcInputData.v_[i + 12] = buf.jointStates.rf_vel.value[i];
            mpcInputData.v_[i + 15] = buf.jointStates.rh_vel.value[i];
        }
        // ContactStates
        mpcInputData.numOfStance_ = 0;
        mpcInputData.stance_[0] = buf.contact.lf;
        mpcInputData.stance_[1] = buf.contact.rf;
        mpcInputData.stance_[2] = buf.contact.lh;
        mpcInputData.stance_[3] = buf.contact.rh;
        mpcInputData.stance_bool_[0] = buf.contact.lf;
        mpcInputData.stance_bool_[1] = buf.contact.rf;
        mpcInputData.stance_bool_[2] = buf.contact.lh;
        mpcInputData.stance_bool_[3] = buf.contact.rh;
        for(uint i = 0; i < numOfContactPoint; i++){
            if(mpcInputData.stance_[i]){
                mpcInputData.numOfStance_++;
            }
        }
        // Pose
        baseQuat = buf.base_orientation_world;
        baseRPY = yawTotalCounter.quaternionToTotalRad(baseQuat);
        // Yaw Pitch Roll
        mpcInputData.q_[3] = baseRPY[2];
        mpcInputData.q_[4] = baseRPY[1];
        mpcInputData.q_[5] = baseRPY[0];
        //Position
        mpcInputData.q_.head(3) = buf.base_pos_world;
        // mpcInputData.q_[2] = 0;
        // Omega

        //實物中這裡全是body是不行的
        matrix3_t transMatrixInverse = rpyDotTOtwist(mpcInputData.q_[3], mpcInputData.q_[4], mpcInputData.q_[5]).inverse();
        mpcInputData.v_.segment<3>(3) = transMatrixInverse * buf.base_angular_vel_world;
        if(!isnormal(mpcInputData.v_[3])){
            std::cout << " singluarity transMatrix!!!!!!!! \n";
            abort();
            mpcInputData.v_[3] = buf.base_angular_vel_world[2];
            mpcInputData.v_[4] = buf.base_angular_vel_world[1];
            mpcInputData.v_[5] = buf.base_angular_vel_world[0]; 
        }
        // mpcInputData.v_[3] = buf.base_angular_vel_world[2];
        // mpcInputData.v_[4] = buf.base_angular_vel_world[1];
        // mpcInputData.v_[5] = buf.base_angular_vel_world[0]; 
        // Velocity
        mpcInputData.v_.head(3) = buf.base_linear_vel_world;
        // Terrain
        #ifdef USE_TERRAIN
        mpcInputData.terrainRotMat = buf.terrain_orientation.toRotationMatrix();
        mpcInputData.terrain_params = buf.terrain_params;
        #endif

        // mpc_reset********************************************************************
        if(!isReset) {
            // Initial state
            vector_t state = vector_t::Zero(6 + dofOfRobot);
            vector_t input = vector_t::Zero(3 * numOfContactPoint + numOfActuatedJoint);

            state.tail(dofOfRobot) = mpcInputData.q_;
            // state(8) = 0.51; //z = 0.48
            state.tail(12) << -0.007, -0.84, 1.584, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584;

            // Initial command
            TargetTrajectories initTargetTrajectories({0.0}, {state}, {input});

            ocs2_msgs::reset resetSrv;
            resetSrv.request.reset = static_cast<uint8_t>(true);
            resetSrv.request.targetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

            while (!mpcResetServiceClient_.waitForExistence(ros::Duration(5.0)) && ::ros::ok() && ::ros::master::check()) {
                ROS_ERROR_STREAM("Failed to call  service to reset MPC, retrying...");
            }

            mpcResetServiceClient_.call(resetSrv);
            ROS_INFO_STREAM("MPC node has been reset.");

            isReset = true;
        }

        // mpc_msg******************************************************************
        // mpc_state_msg
        mpc_input_msg.state.value.resize(6 + dofOfRobot);
        const auto& model = pinocchioInterfacePtr->getModel();
        auto& data = pinocchioInterfacePtr->getData();
        pinocchio::forwardKinematics(model, data, mpcInputData.q_, mpcInputData.v_);
        // pinocchio::computeCentroidalMomentum(model, data);

        const auto& Ag = pinocchio::computeCentroidalMap(model, data, mpcInputData.q_);
        const auto& Hcom = Ag * mpcInputData.v_;
        pinocchio::computeTotalMass(model, data);
        // Centroidal Momtentum
        std::cout << "robot mass data.mass[0]: " << data.mass[0] << std::endl;
        for(uint i = 0; i < 3; i++){
            mpc_input_msg.state.value[i] = Hcom[i] / data.mass[0]; // data.hg.linear()[i];
            mpc_input_msg.state.value[i + 3] = Hcom[i+3] / data.mass[0];  // data.hg.angular()[i];
        }
        // XYZ RPY
        for(uint i = 0; i < 6; i++){
            mpc_input_msg.state.value[i + 6] = mpcInputData.q_[i];
        }
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
                    mpc_input_msg.input.value[index + j] = 0;//mpcInputData.grf_[3 * i + j];
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
            mpc_input_msg.input.value[3 * mpcInputData.numOfStance_ + i] = mpcInputData.v_[6 + i];
        }

        // mpc_time_msg
        mpc_input_msg.time = buf.time_stamp;

        // mpc_mode_msg
        mpc_input_msg.mode = int(legged_robot::stanceLeg2ModeNumber(mpcInputData.stance_bool_));

        // mpc_terrain_msg
        #ifdef USE_TERRAIN
        Vec31<double> ref_params;
        double ref_base_height(0.5);
        double z_ref;

        ref_params[0] = mpcInputData.terrain_params[0];
        ref_params[1] = mpcInputData.,[1];
        ref_params[2] = mpcInputData.terrain_params[2] - sqrt(mpcInputData.terrain_params[0] * mpcInputData.terrain_params[0] + mpcInputData.terrain_params[1] * mpcInputData.terrain_params[1] + 1) * ref_base_height;

        z_ref = -(ref_params[0] * mpcInputData.q_[0] + ref_params[1] * mpcInputData.q_[1] + ref_params[2]);

        if(firstRun) {
            mpcInputData_pre.terrainRotMat = mpcInputData.terrainRotMat;
            mpcInputData_pre.terrain_params = mpcInputData.terrain_params;
            z_ref_pre = z_ref;

            firstRun = false;
        }

        // if(abs(z_ref_pre - z_ref) <= 0.07) {
            mpc_input_msg.param.value.resize(12); // 0-8: Rotation Matrix    9-11: Terrain Plane Function
            mpc_input_msg.param.value[0] = mpcInputData.terrainRotMat(0,0);
            mpc_input_msg.param.value[1] = mpcInputData.terrainRotMat(0,1);
            mpc_input_msg.param.value[2] = mpcInputData.terrainRotMat(0,2);

            mpc_input_msg.param.value[3] = mpcInputData.terrainRotMat(1,0);
            mpc_input_msg.param.value[4] = mpcInputData.terrainRotMat(1,1);
            mpc_input_msg.param.value[5] = mpcInputData.terrainRotMat(1,2);

            mpc_input_msg.param.value[6] = mpcInputData.terrainRotMat(2,0);
            mpc_input_msg.param.value[7] = mpcInputData.terrainRotMat(2,1);
            mpc_input_msg.param.value[8] = mpcInputData.terrainRotMat(2,2);
            mpc_input_msg.param.value[9] = mpcInputData.terrain_params[0];
            mpc_input_msg.param.value[10] = mpcInputData.terrain_params[1];
            mpc_input_msg.param.value[11] = mpcInputData.terrain_params[2];

            z_ref_pre = z_ref;
            mpcInputData_pre.terrainRotMat  = mpcInputData.terrainRotMat;
            mpcInputData_pre.terrain_params = mpcInputData.terrain_params;
        // }
        // else {
        //     mpc_input_msg.param.value.resize(12); // 0-8: Rotation Matrix    9-11: Terrain Plane Function
        //     mpc_input_msg.param.value[0] = mpcInputData_pre.terrainRotMat(0,0);
        //     mpc_input_msg.param.value[1] = mpcInputData_pre.terrainRotMat(0,1);
        //     mpc_input_msg.param.value[2] = mpcInputData_pre.terrainRotMat(0,2);

        //     mpc_input_msg.param.value[3] = mpcInputData_pre.terrainRotMat(1,0);
        //     mpc_input_msg.param.value[4] = mpcInputData_pre.terrainRotMat(1,1);
        //     mpc_input_msg.param.value[5] = mpcInputData_pre.terrainRotMat(1,2);

        //     mpc_input_msg.param.value[6] = mpcInputData_pre.terrainRotMat(2,0);
        //     mpc_input_msg.param.value[7] = mpcInputData_pre.terrainRotMat(2,1);
        //     mpc_input_msg.param.value[8] = mpcInputData_pre.terrainRotMat(2,2);

        //     mpc_input_msg.param.value[9]  = mpcInputData_pre.terrain_params[0];
        //     mpc_input_msg.param.value[10] = mpcInputData_pre.terrain_params[1];
        //     mpc_input_msg.param.value[11] = mpcInputData_pre.terrain_params[2];
        // }
        #endif

        mpc_terrain_sync_input_msg.a = buf.terrainEstData.terrainParams[0];
        mpc_terrain_sync_input_msg.b = buf.terrainEstData.terrainParams[1];
        mpc_terrain_sync_input_msg.d = buf.terrainEstData.terrainParams[2];

        std::cout << "buf.terrainEstData.terrainParams: " << buf.terrainEstData.terrainParams.transpose() << std::endl;

        mpc_terrain_sync_input_msg.quaternion.w = buf.terrainEstData.terrainQuat.w();
        mpc_terrain_sync_input_msg.quaternion.x = buf.terrainEstData.terrainQuat.x();
        mpc_terrain_sync_input_msg.quaternion.y = buf.terrainEstData.terrainQuat.y();
        mpc_terrain_sync_input_msg.quaternion.z = buf.terrainEstData.terrainQuat.z();

        mpc_terrain_sync_input_msg.feetHeight[0] = buf.terrainEstData.feetHeight[0];
        mpc_terrain_sync_input_msg.feetHeight[1] = buf.terrainEstData.feetHeight[1];
        mpc_terrain_sync_input_msg.feetHeight[2] = buf.terrainEstData.feetHeight[2];
        mpc_terrain_sync_input_msg.feetHeight[3] = buf.terrainEstData.feetHeight[3];

        mpc_terrain_sync_input.publish(mpc_terrain_sync_input_msg);
        mpc_input.publish(mpc_input_msg);

        if(debug){
            std::cout << "publish msg done!" << std::endl;
            std::cout << "____________MPC Input Massage:____MPC_PRONTO_CONVERSION_________" << std::endl;
            std::cout << "MPC State:____________ " << std::endl;
            std::cout << "Centrodial Momentum: x y z roll pitch yaw" <<std::endl;
            for(uint i = 0; i < 6; i++){
                std::cout << mpc_input_msg.state.value[i] << " ";
                std::cout << mpc_input_msg.state.value[i] << " ";
            }
            std::cout << "\nBody Pose: x y z yaw pitch roll" << std::endl;
            std::cout << "\nBody Pose: x y z yaw pitch roll" << std::endl;
            for(uint i = 0; i < 6; i++){
                std::cout << mpc_input_msg.state.value[i + 6] << " ";
                std::cout << mpc_input_msg.state.value[i + 6] << " ";
            }
            std::cout << "\nActuated Joints:" << std::endl;
            std::cout << "\nActuated Joints:" << std::endl;
            for(uint i = 0; i < numOfActuatedJoint; i++){
                std::cout << mpc_input_msg.state.value[i + 12] << " ";
                std::cout << mpc_input_msg.state.value[i + 12] << " ";
            }
            std::cout << "\nMPC Input:___________ " << std::endl;
            std::cout << "\nMPC Input:___________ " << std::endl;
            std::cout << "Contact Point Forces: " << std::endl;
            for(uint i = 0; i < mpc_input_msg.input.value.size() - numOfActuatedJoint; i++){
                std::cout << mpc_input_msg.input.value[i] << " ";
            }
            std::cout << "\nActuated Joints speed: " << std::endl;  
            for(uint i = 0; i < numOfActuatedJoint; i++){
                std::cout << mpc_input_msg.input.value[mpc_input_msg.input.value.size() - 12 + i] << " ";
            }
            std::cout << "\nGait Mode:____________" << std::endl;
            std::cout << int(mpcInputData.stance_bool_[0]) << " ";
            std::cout << int(mpcInputData.stance_bool_[1]) << " ";
            std::cout << int(mpcInputData.stance_bool_[2]) << " ";
            std::cout << int(mpcInputData.stance_bool_[3]) << "\n";
            std::cout << double(mpc_input_msg.mode) << std::endl;
            std::cout << "terrain Parameters: " << buf.terrainEstData.terrainParams.transpose() << std::endl;
            // std::cout << "time: " << mpc_input_msg.time << std::endl;
        }
        // rate.sleep();
    }

    close(rec_fd);

    return 0;
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

matrix3_t rpyDotTOtwist(double theta_z, double theta_y, double theta_x){
    matrix3_t translation_Matrix;

    translation_Matrix << 0, -sin(theta_z), cos(theta_y) * cos(theta_z),
                          0, cos(theta_z), cos(theta_y) * sin(theta_z),
                          1, 0 , -sin(theta_y);
                          
    return translation_Matrix;
}
