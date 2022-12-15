#pragma GCC optimize(2)
// pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
// c++
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
// ros
#include "ros/ros.h"
#include "ros/node_handle.h"
// ocs2
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_state.h>
#include <ocs2_msgs/mpc_input.h>
#include <ocs2_msgs/mpc_wbc_traj.h>
#include <ocs2_msgs/mpc_wbc_conversion.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

using namespace ocs2;

// UDP
#define SEND_PORT 1111
#define SEND_IP "127.0.0.1"
int send_fd;
struct sockaddr_in send_aadr;
int useless;

// Data Struct For WBC
// This class contains the primal mpc problem's solution.
struct mpcPolicyData {
    scalar_array_t timeTrajectory_;
    vector_array_t stateTrajectory_;
    vector_array_t inputTrajectory_;
    ModeSchedule modeSchedule_;
};
// MPC OUTPUT FOR UDP
#define LENGTH 10
size_t N_times = LENGTH;
using vector_foot_t = Eigen::Matrix<Eigen::Matrix<Eigen::Matrix<float, 3, 1 >,4, 1>, LENGTH, 1>;
using vector_base_t = Eigen::Matrix <Eigen::Matrix<float, 6, 1>, LENGTH, 1>;
using vector_joint_t = Eigen::Matrix <Eigen::Matrix<float, 12, 1>, LENGTH, 1>;
struct conversionData{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    vector_foot_t swingFeetPosition; //lf lh rf rh
    vector_foot_t swingFeetVelocity; //lf lh rf rh
    vector_foot_t swingFeetAcceleration;  //lf lh rf rh
    Eigen::Vector4f firstGait; //lf lh rf rh
    Eigen::Vector4f secondGait; //lf lh rf rh
    Eigen::Vector4f thirdGait; //lf lh rf rh
    Eigen::Vector2f switchTime;
    vector_base_t basePosition;
    vector_base_t baseVelocity;
    vector_base_t baseAcceleration;
    vector_joint_t jointPos;
    vector_joint_t jointVel;
    vector_joint_t jointAcc;
    Eigen::Matrix<float, LENGTH,1> stateTime;
};
 
// Global Variables
mpcPolicyData mpcData;
conversionData wbcInterfaceData;
std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
legged_robot::ModelSettings modelSettings;
uint8_t numOfActuatedJoint(12);
uint8_t dimOfContactForce(12);
uint8_t dofOfRobot(18);
size_t N_contactPoint(4);
vector_array_t q; // Generalized Coordinates
vector_array_t v; // Generalized Velocities
vector_array_t a; // Generalized Accelerations
matrix_array_t InverseAb; // Inverse of Centroidal momentum matrix (Base patition)
matrix_array_t Aj; // Centroidal momentum matrix (joint patition)

// Functions
void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg);
void KinematicDynamicSetup(std::string& urdfFilePath);
void FiniteDifferencesActuatedJointAcc();
void getGeneralizedCoordinates();
void getGeneralizedVelocities();
void DesiredTrajectoriesForWBC();
Eigen::Matrix<float, 4, 1> modeNumber2StanceLeg_WBC(const size_t& modeNumber);
void pseudoInverse(matrix_t const& matrix, double sigmaThreshold, matrix_t& invMatrix);
Eigen::Matrix<double, 3, 3> rpyDotTOtwist(double yaw, double pitch, double roll);
Eigen::Matrix<double, 3, 3> rpyDotTOtwistDot(double yaw, double pitch, double roll, double yaw_dot, double pitch_dot, double roll_dot);


int main(int argc, char **argv)
{
    // UDP

    send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(send_fd < 0){
        printf("create socket fail!\n");
        return -1;
    }
    memset(&send_aadr, 0 , sizeof(send_aadr));
    send_aadr.sin_family = AF_INET;
    send_aadr.sin_addr.s_addr = inet_addr(SEND_IP);
    send_aadr.sin_port = htons(SEND_PORT);

    // ROS
    std::vector<std::string> programArgs{};
    ::ros::removeROSArgs(argc, argv, programArgs);
    ros::init(argc, argv, "MPC_WBC_ConversionsUDP");
    ros::NodeHandle nh;
    ros::Subscriber mpcPolicySubscriber;

    // URDF Model -> Pinocchio Model
    std::string urdfFilePath;
    // if (!ros::param::get("/legged_robot_description", urdfFilePath)) {
    //   std::cerr << "Param " << "/legged_robot_description" << " not found; unable to generate urdf" << std::endl;
    // }
    nh.getParam("/urdfFile", urdfFilePath);
    KinematicDynamicSetup(urdfFilePath);
    // MPC Policy Subscriber
    mpcPolicySubscriber = nh.subscribe("/legged_robot_mpc_policy", 1, &mpcPolicyCallback);

    //FOR ROS
    // while(nh.ok()){
    //     ros::spinOnce();
    // }

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    close(send_fd);

    return 0;
}

void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg) {
        
        size_t N_modeSequence = msg->modeSchedule.modeSequence.size(); // Gait Mode Sequence
        //Resize MPC Policy Data 
        // std::cout << "traj length: " << msg->timeTrajectory.size() << "\n";
        if (msg->timeTrajectory.size() > LENGTH){
          N_times = LENGTH;
        }
        else{
          N_times = msg->timeTrajectory.size();
        }
        


        mpcData.timeTrajectory_.clear();
        mpcData.timeTrajectory_.resize(N_times);
        mpcData.stateTrajectory_.clear();
        mpcData.stateTrajectory_.resize(N_times);
        mpcData.inputTrajectory_.clear();
        mpcData.inputTrajectory_.resize(N_times);
        mpcData.modeSchedule_.modeSequence.clear();
        mpcData.modeSchedule_.modeSequence.resize(N_modeSequence);
        mpcData.modeSchedule_.eventTimes.clear();
        mpcData.modeSchedule_.eventTimes.resize(N_modeSequence - 1);

        //Resize MPC_WBC_Conversion Data
        q.clear();
        q.resize(N_times);
        v.clear();
        v.resize(N_times);
        a.clear();
        a.resize(N_times);
        InverseAb.clear();
        InverseAb.resize(N_times);
        Aj.clear();
        Aj.resize(N_times);

        // std::cerr << "\n[dqwang: mpcPolicyCallback] Resize MPC Policy Data Done!\n";
        // std::cerr << "\n[dqwang: mpcPolicyCallback] msg->timeTrajectory.size() " << msg->timeTrajectory.size() << "\n";

        // Copy MPC Policy Data from msg
        // time
        for(size_t k = 0; k < N_times; k++) {
            mpcData.timeTrajectory_[k] = msg->timeTrajectory[k];
            // std::cerr << "\n[dqwang: mpcPolicyCallback] mpcData.timeTrajectory_ \n" << mpcData.timeTrajectory_[k] << "\n";
        } 
        
        // state
        for(size_t k = 0; k < N_times; k++){
            mpcData.stateTrajectory_[k].resize(msg->stateTrajectory[k].value.size());
            for(size_t j = 0; j < msg->stateTrajectory[k].value.size(); j++){
                mpcData.stateTrajectory_[k][j] = msg->stateTrajectory[k].value[j];
            }
        } 
        // input
        for(size_t k = 0; k < N_times; k++){
            mpcData.inputTrajectory_[k].resize(msg->inputTrajectory[k].value.size());
            for(size_t j = 0; j < msg->inputTrajectory[k].value.size(); j++){
                mpcData.inputTrajectory_[k][j] = msg->inputTrajectory[k].value[j];
            }
        }

        // std::cerr << "\n[dqwang: mpcPolicyCallback] Copy MPC Policy Data from msg Done!\n";

   

        // contact
        for (uint8_t i = 0; i < N_modeSequence - 1; i++){
            double delta_time = msg->modeSchedule.eventTimes[i] - msg->timeTrajectory[0];
            if(delta_time > 0){
                // wbcInterfaceData.switchTime[0] = delta_time;
                wbcInterfaceData.switchTime[0] = msg->modeSchedule.eventTimes[i];
                wbcInterfaceData.firstGait = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i]);

                if (i+2 < N_modeSequence){
                    // wbcInterfaceData.switchTime[1] = msg->modeSchedule.eventTimes[i+1] - msg->timeTrajectory[0];
                    wbcInterfaceData.switchTime[1] = msg->modeSchedule.eventTimes[i+1];
                    wbcInterfaceData.secondGait = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i+1]);
                    wbcInterfaceData.thirdGait  = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i+2]);
                }
                else{
                    // wbcInterfaceData.switchTime[1] = delta_time;
                    wbcInterfaceData.switchTime[1] = msg->modeSchedule.eventTimes[i];
                    wbcInterfaceData.secondGait = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i]);
                    wbcInterfaceData.thirdGait  = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i]);
                    std::cout << "time too close to end\n";
                }
                break;
            }
        }

        // std::cerr << "\n[dqwang: mpcPolicyCallback] Contact Done!\n";

        // time stamp
        for (int i = 0; i < N_times; i++) { 
            wbcInterfaceData.stateTime[i] = mpcData.timeTrajectory_[i];
        }

        // std::cerr << "\n[dqwang: mpcPolicyCallback] time_stamp Done!\n";

        FiniteDifferencesActuatedJointAcc();

        // std::cerr << "\n[dqwang: mpcPolicyCallback] FiniteDifferences Done!\n";

        getGeneralizedCoordinates();

        // std::cerr << "\n[dqwang: mpcPolicyCallback] getGeneralizedCoordinates Done!\n";

        getGeneralizedVelocities();

        // std::cerr << "\n[dqwang: mpcPolicyCallback] getGeneralizedVelocities Done!\n";

        DesiredTrajectoriesForWBC();

        // std::cerr << "\n[dqwang: mpcPolicyCallback] DesiredTrajectoriesForWBC Done!\n";
        std::cerr << "\n[dqwang: mpcPolicyCallback] wbcInterfaceData state times start: " << wbcInterfaceData.stateTime[0] << "\n";

        // std::cerr << "\n[dqwang: mpcPolicyCallback] wbcInterfaceData firstgait: " << wbcInterfaceData.firstGait;
        // std::cerr << "\n[dqwang: mpcPolicyCallback] wbcInterfaceData secondgait: " << wbcInterfaceData.secondGait;

        int res = sendto(send_fd, &wbcInterfaceData, sizeof(wbcInterfaceData), 0, (struct sockaddr*)&send_aadr, (socklen_t)sizeof(send_aadr));
        auto now = std::chrono::steady_clock::now();
        static  std::chrono::steady_clock::time_point t1;
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(now - t1).count();
        std::cout << "MPC_WBC WAKE TIME: " << us << "us" << std::endl;
        t1 = now;
        // std::chrono::steady_clock::time_point send_tp = std::chrono::steady_clock::now();
        // std::cerr << "send to time" 
        // << std::chrono::duration_cast<std::chrono::milliseconds>( .time_since_epoch()).count() << "\n";
}

void KinematicDynamicSetup(std::string& urdfFilePath){
    // PinocchioInterface
    pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFilePath, modelSettings.jointNames)));
}

void FiniteDifferencesActuatedJointAcc(){
    scalar_t delta_t;

    // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] 1 Done!\n";
    // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] N_times " << N_times << "\n";

    for(size_t k = 0; k < N_times; k++){
        a[k].resize(dofOfRobot); 
        // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] dofOfRobot " << dofOfRobot << "\n";
        // First Time
        if(k == 0){
            size_t dim_inputTrajectory_k = mpcData.inputTrajectory_[k].size();
            size_t dim_inputTrajectory_k_plus = mpcData.inputTrajectory_[k + 1].size();
            delta_t  = mpcData.timeTrajectory_[k] - mpcData.timeTrajectory_[k + 1];
            if(abs(delta_t) < 0.001){
                delta_t -= 0.001;
            }

            // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] delta_t " << delta_t << "\n";
            // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] N_times " << N_times << "\n";

            for(size_t j = 0; j < numOfActuatedJoint; j++){
                a[k][j + 6] = (mpcData.inputTrajectory_[k][j + dim_inputTrajectory_k - numOfActuatedJoint] - mpcData.inputTrajectory_[k + 1][j + dim_inputTrajectory_k_plus - numOfActuatedJoint])/
                              (delta_t);
                // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] mpcData.inputTrajectory_[k][j + dim_inputTrajectory_k - numOfActuatedJoint]  " << j << " " <<mpcData.inputTrajectory_[k][j + dim_inputTrajectory_k - numOfActuatedJoint] << "\n";
                // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] 1 Done!\n";
            }

            // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] First Time Done!\n";
        }

        // Last Time
        else if(k == N_times - 1){
            size_t dim_inputTrajectory_k_minus = mpcData.inputTrajectory_[k - 1].size();
            size_t dim_inputTrajectory_k = mpcData.inputTrajectory_[k].size();
            delta_t  = mpcData.timeTrajectory_[k - 1] - mpcData.timeTrajectory_[k];
            if(abs(delta_t) < 0.001){
                delta_t -= 0.001;
            }
            for(size_t j = 0; j < numOfActuatedJoint; j++){
                a[k][j + 6] = (mpcData.inputTrajectory_[k - 1][j + dim_inputTrajectory_k_minus - numOfActuatedJoint] - mpcData.inputTrajectory_[k][j + dim_inputTrajectory_k - numOfActuatedJoint])/
                              (delta_t);
            }
            // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] Last Time Done!\n";
        }

        //Middle Time
        else{
            size_t dim_inputTrajectory_k_minus = mpcData.inputTrajectory_[k - 1].size();
            size_t dim_inputTrajectory_k_plus = mpcData.inputTrajectory_[k + 1].size();
            delta_t  = mpcData.timeTrajectory_[k - 1] - mpcData.timeTrajectory_[k];
            if(abs(delta_t) < 0.001){
                delta_t -= 0.001;
            }
            for(size_t j = 0; j < numOfActuatedJoint; j++){
                a[k][j + 6] = (mpcData.inputTrajectory_[k - 1][j + dim_inputTrajectory_k_minus - numOfActuatedJoint] - mpcData.inputTrajectory_[k][j + dim_inputTrajectory_k_plus - numOfActuatedJoint])/
                              (delta_t);
            }        
        }

        // std::cerr << "\n[dqwang: FiniteDifferencesActuatedJointAcc] Middle Time Done!\n";
    }
}

void getGeneralizedCoordinates(){
    N_times = mpcData.timeTrajectory_.size();
    for(size_t k = 0; k < N_times; k++){
        q[k].resize(dofOfRobot);
        // Base--6DOF
        for(size_t j = 0; j < 6; j++){
            q[k][j] = mpcData.stateTrajectory_[k][6 + j];
        }
        // Actuated joints
        for(size_t j = 0; j < numOfActuatedJoint; j++){
            q[k][j + 6] = mpcData.stateTrajectory_[k][12 + j];
        }
    }
}

void getGeneralizedVelocities(){
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();
    pinocchio::computeTotalMass(model, data);

    for(size_t k = 0; k < N_times; k++){
        v[k].resize(dofOfRobot);
        // Actuated joints
        for(size_t j = 0; j < numOfActuatedJoint; j++){
            v[k][j + 6] = mpcData.inputTrajectory_[k][j + dimOfContactForce];
        }
        // Base
        const auto& Ag = pinocchio::computeCentroidalMap(model, data, q[k]); // Computes the Centroidal Momentum Matrix
        pseudoInverse(Ag.leftCols(6), 0.0001, InverseAb[k]); // InverseAb
        Aj[k] = Ag.rightCols(numOfActuatedJoint); // Aj
        v[k].head(6) = InverseAb[k] * (mpcData.stateTrajectory_[k].head(6) * data.mass[0] - Aj[k] * v[k].tail(numOfActuatedJoint));
    }
}

void DesiredTrajectoriesForWBC(){
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();

    pinocchio::computeTotalMass(model, data);
    for(size_t k = 0; k < N_times; k++){
      pinocchio::computeJointJacobians(model, data, q[k]);
      pinocchio::computeJointJacobiansTimeVariation(model, data, q[k], v[k]);
      pinocchio::updateFramePlacements(model, data);

      // Base Position
      wbcInterfaceData.basePosition[k][0] = q[k][0]; // x
      wbcInterfaceData.basePosition[k][1] = q[k][1]; // y
      wbcInterfaceData.basePosition[k][2] = q[k][2]; // z
      wbcInterfaceData.basePosition[k][3] = q[k][5]; // roll
      wbcInterfaceData.basePosition[k][4] = q[k][4]; // pitch
      wbcInterfaceData.basePosition[k][5] = q[k][3]; // yaw

      // Base Velocity
      wbcInterfaceData.baseVelocity[k][0] = v[k][0]; //x
      wbcInterfaceData.baseVelocity[k][1] = v[k][1]; //y
      wbcInterfaceData.baseVelocity[k][2] = v[k][2]; //z
      // wbcInterfaceData.baseVelocity[k][3] = v[k][5]; // roll        
      // wbcInterfaceData.baseVelocity[k][4] = v[k][4]; // pitch      
      // wbcInterfaceData.baseVelocity[k][5] = v[k][3]; // yaw  
      //这里的顺序不用换了
      wbcInterfaceData.baseVelocity[k].tail(3) = (rpyDotTOtwist(q[k][3], q[k][4], q[k][5]) * v[k].segment(3, 3)).cast<float>();//X Y Z // to check

      
      // Contact Point Position
      for(size_t j = 0; j < N_contactPoint; j++){
          wbcInterfaceData.swingFeetPosition[k][j] = data.oMf[model.getBodyId(modelSettings.contactNames3DoF[j])].translation().cast<float>();
      }
      // Base Acceleration
      pinocchio::computeCentroidalMapTimeVariation(model, data, q[k], v[k]); //the time derivative of the Centroidal Momentum Matrix
      vector_t hDot = vector_t::Zero(6);
      for(size_t j = 0; j < N_contactPoint; j++){
          // XYZ centroidal dynamics
          hDot.head(3) += mpcData.inputTrajectory_[k].segment(3*j, 3);
          Eigen::Matrix<double, 3, 1> f = mpcData.inputTrajectory_[k].segment(3*j, 3);
          Eigen::Matrix<double, 3, 1> r = wbcInterfaceData.swingFeetPosition[k][j].cast<double>() - q[k].head(3);
          hDot.tail(3) += r.cross(f);
      }
      hDot[2] -= data.mass[0] * 9.81f;
      Eigen::Matrix<double, 6, 1> q_base_ddot;
      q_base_ddot = InverseAb[k] * (hDot - data.dAg * v[k] - Aj[k] * a[k].tail(numOfActuatedJoint));
      a[k].head(6) = q_base_ddot;
      wbcInterfaceData.baseAcceleration[k][0] = a[k][0]; //x
      wbcInterfaceData.baseAcceleration[k][1] = a[k][1]; //y
      wbcInterfaceData.baseAcceleration[k][2] = a[k][2]; //z
      // wbcInterfaceData.baseAcceleration[k][3] = a[k][5]; // roll   
      // wbcInterfaceData.baseAcceleration[k][4] = a[k][4]; // pitch   
      // wbcInterfaceData.baseAcceleration[k][5] = a[k][3]; // yaw 
      wbcInterfaceData.baseAcceleration[k].tail(3) = 
                    (rpyDotTOtwistDot(q[k][3], q[k][4], q[k][5], v[k][3], v[k][4], v[k][5]) * a[k].segment(3, 3)).cast<float>();//X Y Z // to check


      // Contact Point Velocity and Acceleration
      for(size_t j = 0; j < N_contactPoint; j++){
          matrix_t jacobianContactPoint = matrix_t::Zero(6, dofOfRobot);
          matrix_t jacobianDotContactPoint = matrix_t::Zero(6, dofOfRobot);
          pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings.contactNames3DoF[j]), pinocchio::LOCAL_WORLD_ALIGNED, jacobianContactPoint);   
          pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId(modelSettings.contactNames3DoF[j]), pinocchio::LOCAL_WORLD_ALIGNED, jacobianDotContactPoint);
          // Contact Point Velocity
          wbcInterfaceData.swingFeetVelocity[k][j] = (jacobianContactPoint * v[k]).head(3).cast<float>();
          // Contact Point Acceleration rdotdot = J*qdotdot + Jdot*qdot
          wbcInterfaceData.swingFeetAcceleration[k][j] = (jacobianContactPoint * a[k] + jacobianDotContactPoint * v[k]).head(3).cast<float>();
      } 

      // Desired Joint Position and Velocity
      for (int j(0); j < 12; j++) {
        wbcInterfaceData.jointPos[k][j] = q[k][6 + j];
        wbcInterfaceData.jointVel[k][j] = v[k][6 + j];
        wbcInterfaceData.jointAcc[k][j] = a[k][6 + j];
      }
    }
}

Eigen::Matrix<float, 4, 1> modeNumber2StanceLeg_WBC(const size_t& modeNumber){
    Eigen::Matrix<float, 4, 1> stanceLegs;
    switch (modeNumber)
    {
        case 0:
          stanceLegs << false, false, false, false;
          break;  // 0:  0-leg-stance
        case 1:
          stanceLegs << false, false, false, true;
          break;  // 1:  RH
        case 2:
          stanceLegs << false, false, true, false;
          break;  // 2:  LH
        case 3:
          stanceLegs << false, false, true, true;
          break;  // 3:  RH, LH
        case 4:
          stanceLegs << false, true, false, false;
          break;  // 4:  RF
        case 5:
          stanceLegs << false, true, false, true;
          break;  // 5:  RF, RH
        case 6:
          stanceLegs << false, true, true, false;
          break;  // 6:  RF, LH
        case 7:
          stanceLegs << false, true, true, true;
          break;  // 7:  RF, LH, RH
        case 8:
          stanceLegs << true, false, false, false;
          break;  // 8:  LF,
        case 9:
          stanceLegs << true, false, false, true;
          break;  // 9:  LF, RH
        case 10:
          stanceLegs << true, false, true, false;
          break;  // 10: LF, LH
        case 11:
          stanceLegs << true, false, true, true;
          break;  // 11: LF, LH, RH
        case 12:
          stanceLegs << true, true, false, false;
          break;  // 12: LF, RF
        case 13:
          stanceLegs << true, true, false, true;
          break;  // 13: LF, RF, RH
        case 14:
          stanceLegs << true, true, true, false;
          break;  // 14: LF, RF, LH
        case 15:
          stanceLegs << true, true, true, true;
          break;  // 15: 4-leg-stance
    }
    return stanceLegs;
}

void pseudoInverse(matrix_t const& matrix, double sigmaThreshold, matrix_t& invMatrix) {
  if (  (1 == matrix.rows()) && (1 == matrix.cols()) ) {
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }

  Eigen::JacobiSVD<matrix_t> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  matrix_t invS;
  invS = matrix_t::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      printf("sigular value is too small: %f\n",svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
}

Eigen::Matrix<double, 3, 3> rpyDotTOtwist(double theta_z, double theta_y, double theta_x){
    Eigen::Matrix<double, 3, 3> translation_Matrix;

    translation_Matrix << 0, -sin(theta_z), cos(theta_y) * cos(theta_z),
                          0, cos(theta_z), cos(theta_y) * sin(theta_z),
                          1, 0 , -sin(theta_y);
                          
    return translation_Matrix;
}

Eigen::Matrix<double, 3, 3> rpyDotTOtwistDot(double yaw, double pitch, double roll, double yaw_dot, double pitch_dot, double roll_dot){
    Eigen::Matrix<double, 3, 3> translation_Matrix_Dot;

    translation_Matrix_Dot << 0, -cos(yaw) * yaw_dot, -sin(pitch) * cos(yaw) * pitch_dot - cos(pitch) * sin(yaw) * yaw_dot,
                              0, -sin(yaw) * yaw_dot, -sin(pitch) * sin(yaw) * pitch_dot + cos(pitch) * cos(yaw) * yaw_dot,
                              0, 0, -cos(pitch) * pitch_dot;
    return translation_Matrix_Dot;
}