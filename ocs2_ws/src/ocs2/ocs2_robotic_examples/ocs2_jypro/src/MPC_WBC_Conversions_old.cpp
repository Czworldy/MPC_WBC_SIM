// #pragma GCC optimize(2)

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

#include <fstream>
#include <queue>


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

#include <chrono>


using namespace ocs2;

// This class contains the primal mpc problem's solution.
struct mpcPolicyData {
    scalar_array_t timeTrajectory_;
    vector_array_t stateTrajectory_;
    vector_array_t inputTrajectory_;
    ModeSchedule modeSchedule_;
};

struct conversionData{
    vector_array2_t swingFeetPosition;
    vector_array2_t swingFeetVelocity;
    vector_array2_t swingFeetAcceleration;
    vector_array_t stanceFeet;
    Eigen::Vector2d switchTime;
    vector_array_t basePosition;
    vector_array_t baseVelocity;
    vector_array_t baseAcceleration;
};

// Global Variables
mpcPolicyData mpcData;
std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
legged_robot::ModelSettings modelSettings;
uint8_t numOfActuatedJoint(12);
uint8_t dimOfContactForce(12);
uint8_t dofOfRobot(18);

conversionData wbcInterfaceData;
vector_array_t q; // Generalized Coordinates
vector_array_t v; // Generalized Velocities
vector_array_t a; // Generalized Accelerations
matrix_array_t InverseAb; // Inverse of Centroidal momentum matrix (Base patition)
matrix_array_t Aj; // Centroidal momentum matrix (joint patition)

ocs2_msgs::mpc_wbc_conversion wbcMsg;
bool wbcMsgisdone = false;





// Functions
void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg);
void KinematicDynamicSetup(const ::urdf::ModelInterfaceSharedPtr& urdfTree);
void FiniteDifferencesActuatedJointAcc();
void getGeneralizedCoordinates();
void getGeneralizedVelocities();
void DesiredTrajectoriesForWBC();
Eigen::Matrix<double, 4, 1> modeNumber2StanceLeg_WBC(const size_t& modeNumber);
void pseudoInverse(matrix_t const& matrix, double sigmaThreshold, matrix_t& invMatrix);
Eigen::Matrix<double, 3, 3> rpyDotTOtwist(double yaw, double pitch, double roll);
Eigen::Matrix<double, 3, 3> rpyDotTOtwistDot(double yaw, double pitch, double roll, double yaw_dot, double pitch_dot, double roll_dot);


int main(int argc, char **argv)
{
    std::vector<std::string> programArgs{};
    ::ros::removeROSArgs(argc, argv, programArgs);

    ros::init(argc, argv, "MPC_WBC_Conversions");
    // ros::NodeHandle nh;
    ros::NodeHandle nh;
    ros::Publisher mpc_wbcPublisher = nh.advertise<ocs2_msgs::mpc_wbc_conversion>("/mpc_wbcPublisher", 1);
    ros::Subscriber mpcPolicySubscriber;
    ros::Rate rate(100);

    std::queue<ocs2_msgs::mpc_wbc_conversion> wbcMsgBuffer;
    

    // URDF Model -> Pinocchio Model
    std::string urdfString;
    if (!ros::param::get("/legged_robot_description", urdfString)) {
      std::cerr << "Param " << "/legged_robot_description" << " not found; unable to generate urdf" << std::endl;
    }
    KinematicDynamicSetup(urdf::parseURDF(urdfString));

    std::cerr << "____________________________dqwang______________________________________" << std::endl;

    // MPC Policy Subscriber
    mpcPolicySubscriber = nh.subscribe("/legged_robot_mpc_policy", 3, &mpcPolicyCallback);
    
    // spin
    while (ros::ok() && ros::master::check()) {
        ros::spinOnce();
        if(wbcMsgisdone){
            // wbcMsgBuffer.push(wbcMsg);
            mpc_wbcPublisher.publish(wbcMsg);
            // if(wbcMsgBuffer.size() > 1){
            //     ocs2_msgs::mpc_wbc_conversion wbcMsgTemp = wbcMsgBuffer.front();

            //     mpc_wbcPublisher.publish(wbcMsgTemp);
            //     // wbcMsgisdone = false;
            //     std::cout << "msg sent____________________________________________________" << wbcMsgBuffer.size() << "\n";
            //     wbcMsgBuffer.pop();
            // }
            // else{
            //     std::cout<<">>>>>>>>>>>>>>Buffer Not Ready:" << wbcMsgBuffer.size() << "\n";
            // }
            wbcMsgisdone = false;
        }
        // rate.sleep();
    }
    return 0;
}

void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg) {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    size_t N_times = msg->timeTrajectory.size(); // Time Horison
    size_t N_modeSequence = msg->modeSchedule.modeSequence.size(); // Gait Mode Sequence

    //Resize MPC Policy Data 
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
    wbcInterfaceData.swingFeetPosition.clear();
    wbcInterfaceData.swingFeetPosition.resize(N_times);
    wbcInterfaceData.swingFeetVelocity.clear();
    wbcInterfaceData.swingFeetVelocity.resize(N_times);
    wbcInterfaceData.swingFeetAcceleration.clear();
    wbcInterfaceData.swingFeetAcceleration.resize(N_times);
    wbcInterfaceData.stanceFeet.clear();
    wbcInterfaceData.stanceFeet.resize(3);
    wbcInterfaceData.basePosition.clear();
    wbcInterfaceData.basePosition.resize(N_times);
    wbcInterfaceData.baseVelocity.clear();
    wbcInterfaceData.baseVelocity.resize(N_times);
    wbcInterfaceData.baseAcceleration.clear();
    wbcInterfaceData.baseAcceleration.resize(N_times);

    wbcMsg.wbcTraj.clear();
    wbcMsg.firstGait.clear();
    wbcMsg.secondGait.clear();
    wbcMsg.thirdGait.clear();
    wbcMsg.switchTime.clear();
    wbcMsg.stateTime.clear();
    wbcMsg.wbcTraj.resize(N_times);
    wbcMsg.firstGait.resize(4);
    wbcMsg.secondGait.resize(4);
    wbcMsg.thirdGait.resize(4);
    wbcMsg.switchTime.resize(2);
    wbcMsg.stateTime.resize(N_times);

    std::cout << "Resize Done!" << std::endl;
    std::cout << "N_times " << N_times << std::endl;

    // Copy MPC Policy Data from msg
    // time
    for(size_t k = 0; k < N_times; k++){
        // mpcData.timeTrajectory_[k] = msg->timeTrajectory[k] - msg->timeTrajectory[0];
        mpcData.timeTrajectory_[k] = msg->timeTrajectory[k];
    } // end of k loop

    // state
    for(size_t k = 0; k < N_times; k++){

        mpcData.stateTrajectory_[k].resize(msg->stateTrajectory[k].value.size());
        for(size_t j = 0; j < msg->stateTrajectory[k].value.size(); j++){
            mpcData.stateTrajectory_[k][j] = msg->stateTrajectory[k].value[j];
        }
    } // end of k loop

    // input
    for(size_t k = 0; k < N_times; k++){
        mpcData.inputTrajectory_[k].resize(msg->inputTrajectory[k].value.size());
        for(size_t j = 0; j < msg->inputTrajectory[k].value.size(); j++){
            mpcData.inputTrajectory_[k][j] = msg->inputTrajectory[k].value[j];
        }
    } // end of k loop

    for (uint8_t i = 0; i < N_modeSequence - 1; i++){
        double delta_time = msg->modeSchedule.eventTimes[i] - msg->timeTrajectory[0];
        if(delta_time > 0){
            // wbcInterfaceData.switchTime[0] = delta_time;
            wbcInterfaceData.switchTime[0] = msg->modeSchedule.eventTimes[i];
            wbcInterfaceData.stanceFeet[0] = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i]);

            if (i+2 < N_modeSequence){
                // wbcInterfaceData.switchTime[1] = msg->modeSchedule.eventTimes[i+1] - msg->timeTrajectory[0];
                wbcInterfaceData.switchTime[1] = msg->modeSchedule.eventTimes[i+1];
                wbcInterfaceData.stanceFeet[1] = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i+1]);
                wbcInterfaceData.stanceFeet[2] = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i+2]);
            }
            else{
                // wbcInterfaceData.switchTime[1] = delta_time;
                wbcInterfaceData.switchTime[1] = msg->modeSchedule.eventTimes[i];
                wbcInterfaceData.stanceFeet[1] = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i]);
                wbcInterfaceData.stanceFeet[2] = modeNumber2StanceLeg_WBC(msg->modeSchedule.modeSequence[i]);
                std::cout << "time too close to end\n";
            }
            break;
        }
    }

    // std::cout << "switchtime" << wbcInterfaceData.switchTime[0] << "\t" << wbcInterfaceData.switchTime[1] << "\n";
    // std::cout << wbcInterfaceData.stanceFeet[0] << "\n____________\n" << wbcInterfaceData.stanceFeet[1] << "\n____________\n" <<wbcInterfaceData.stanceFeet[1] << "\n";

    

    //std::cout << "Copy Done!" << std::endl;
    FiniteDifferencesActuatedJointAcc();
    //std::cout << "FD Done!" << std::endl;
    getGeneralizedCoordinates();
    //std::cout << "GC Done!" << std::endl;
    getGeneralizedVelocities();
    //std::cout << "GD Done!" << std::endl;
    DesiredTrajectoriesForWBC();
    //std::cout << "DT Done!" << std::endl;
    for (int i = 0; i < N_times; i++){  
        for (int j = 0; j < 6; j++){
            wbcMsg.wbcTraj[i].basePos.push_back(q[i][j]);
            // wbcMsg.wbcTraj[i].qdot.push_back(v[i][j]);
            wbcMsg.wbcTraj[i].baseVel.push_back(a[i][j]);
            //wbcMsg.wbcTraj[i].q.push_back(wbcInterfaceData.basePosition[i][j]);
            wbcMsg.wbcTraj[i].baseAcc.push_back(wbcInterfaceData.baseVelocity[i][j]);
            //wbcMsg.wbcTraj[i].qdotdot.push_back(wbcInterfaceData.baseAcceleration[i][j]);
        }
        for (int k = 0; k < modelSettings.contactNames3DoF.size(); k++){
            wbcMsg.wbcTraj[i].swingPos.push_back(wbcInterfaceData.swingFeetPosition[i][k][0]);
            wbcMsg.wbcTraj[i].swingPos.push_back(wbcInterfaceData.swingFeetPosition[i][k][1]);
            wbcMsg.wbcTraj[i].swingPos.push_back(wbcInterfaceData.swingFeetPosition[i][k][2]);

            wbcMsg.wbcTraj[i].swingVel.push_back(wbcInterfaceData.swingFeetVelocity[i][k][0]);
            wbcMsg.wbcTraj[i].swingVel.push_back(wbcInterfaceData.swingFeetVelocity[i][k][1]);
            wbcMsg.wbcTraj[i].swingVel.push_back(wbcInterfaceData.swingFeetVelocity[i][k][2]);

            wbcMsg.wbcTraj[i].swingAcc.push_back(wbcInterfaceData.swingFeetAcceleration[i][k][0]);
            wbcMsg.wbcTraj[i].swingAcc.push_back(wbcInterfaceData.swingFeetAcceleration[i][k][1]);
            wbcMsg.wbcTraj[i].swingAcc.push_back(wbcInterfaceData.swingFeetAcceleration[i][k][2]);
        }
        wbcMsg.stateTime[i] = mpcData.timeTrajectory_[i];
        
    }
    for (uint8_t i = 0; i < 4; i++){
        wbcMsg.firstGait[i] = wbcInterfaceData.stanceFeet[0][i];
        wbcMsg.secondGait[i]= wbcInterfaceData.stanceFeet[1][i];
        wbcMsg.thirdGait[i] = wbcInterfaceData.stanceFeet[2][i];
    }
    wbcMsg.switchTime[0] = wbcInterfaceData.switchTime[0]; wbcMsg.switchTime[1] = wbcInterfaceData.switchTime[1]; 

    // for (int i = 0; i < N_times; i++)
    //     std::cout <<mpcData.timeTrajectory_[i]<<"\t" << wbcMsg.stateTime[i] << std::endl;
    for (int i = 0; i < 4; i++)
        std::cout <<wbcMsg.firstGait[i]<<"\t" << std::endl;
    wbcMsgisdone = true;
    // std::cout <<   << std::endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::cerr << "time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()<< std::endl;

}

void KinematicDynamicSetup(const ::urdf::ModelInterfaceSharedPtr& urdfTree){

    // PinocchioInterface
    pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfTree, modelSettings.jointNames)));
}

void FiniteDifferencesActuatedJointAcc(){
    size_t N_times = mpcData.timeTrajectory_.size();
    scalar_t delta_t;

    for(size_t k = 0; k < N_times; k++){
        a[k].resize(dofOfRobot); 
        //std::cout<<"N:" << N_times<<"\n";
       

        // First Time
        if(k == 0){
            size_t dim_inputTrajectory_k = mpcData.inputTrajectory_[k].size();
            size_t dim_inputTrajectory_k_plus = mpcData.inputTrajectory_[k + 1].size();
            delta_t  = mpcData.timeTrajectory_[k] - mpcData.timeTrajectory_[k + 1];
            if(abs(delta_t) < 0.001){
                delta_t -= 0.001;
            }
            //std::cout << "k == 0" <<"\t"<<k<< std::endl;
            for(size_t j = 0; j < numOfActuatedJoint; j++){
                a[k][j + 6] = (mpcData.inputTrajectory_[k][j + dim_inputTrajectory_k - numOfActuatedJoint] - mpcData.inputTrajectory_[k + 1][j + dim_inputTrajectory_k_plus - numOfActuatedJoint])/
                              (delta_t);
            }
        }
        // Last Time
        
        else if(k == N_times - 1){
            //std::cout << "k == 0" <<"\t"<<k<< std::endl;
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
        }
        //Middle Time
        else{
            size_t dim_inputTrajectory_k_minus = mpcData.inputTrajectory_[k - 1].size();
            size_t dim_inputTrajectory_k_plus = mpcData.inputTrajectory_[k + 1].size();
            //std::cout << "k == 0" <<"\t"<<k<< std::endl;
            delta_t  = mpcData.timeTrajectory_[k - 1] - mpcData.timeTrajectory_[k ];
            if(abs(delta_t) < 0.001){
                delta_t -= 0.001;
            }
            for(size_t j = 0; j < numOfActuatedJoint; j++){
                a[k][j + 6] = (mpcData.inputTrajectory_[k - 1][j + dim_inputTrajectory_k_minus - numOfActuatedJoint] - mpcData.inputTrajectory_[k ][j + dim_inputTrajectory_k_plus - numOfActuatedJoint])/
                              (delta_t);
            }        
        }
    }

}

void getGeneralizedCoordinates(){
    size_t N_times = mpcData.timeTrajectory_.size();
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
    size_t N_times = mpcData.timeTrajectory_.size();
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();

    pinocchio::computeTotalMass(model, data);

    for(size_t k = 0; k < N_times; k++){
        v[k].resize(dofOfRobot);
        //InverseAb[k].resize(6, 6);
        //Aj[k].resize(6, numOfActuatedJoint);

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

std::ofstream swingfootfile;

void DesiredTrajectoriesForWBC(){
    size_t N_times = mpcData.timeTrajectory_.size();
    size_t N_contactPoint = modelSettings.contactNames3DoF.size();
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();

    // swingfootfile.open("/home/dqwang/MPC_WBC/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/WBC/ResultData/mpcfootlf.txt",std::ios::app|std::ios::out);


    pinocchio::computeTotalMass(model, data);
    for(size_t k = 0; k < N_times; k++){
        wbcInterfaceData.swingFeetPosition[k].resize(N_contactPoint);
        wbcInterfaceData.swingFeetVelocity[k].resize(N_contactPoint);
        wbcInterfaceData.swingFeetAcceleration[k].resize(N_contactPoint);

        Eigen::VectorXd qTpl = q[k];
        qTpl.head(6) << 0., 0., 0., 0., 0., 0.;

        pinocchio::computeJointJacobians(model, data, q[k]);
        pinocchio::computeJointJacobiansTimeVariation(model, data, q[k], v[k]);
        
        pinocchio::updateFramePlacements(model, data);

        // Contact Point Position
        for(size_t j = 0; j < N_contactPoint; j++){
            wbcInterfaceData.swingFeetPosition[k][j].resize(3);
            wbcInterfaceData.swingFeetPosition[k][j] = data.oMf[model.getBodyId(modelSettings.contactNames3DoF[j])].translation();
        }
        // swingfootfile <<mpcData.timeTrajectory_[k] << "\t" << wbcInterfaceData.swingFeetPosition[k][0][2] << "\n";
        // mpcData.timeTrajectory_[k] -= mpcData.timeTrajectory_[0];
        // Base Position
        wbcInterfaceData.basePosition[k] = q[k].head(6);
        // Base Velocity
        Eigen::Matrix<double, 3, 1> twist_angular;
        wbcInterfaceData.baseVelocity[k].resize(6);
        wbcInterfaceData.baseVelocity[k].head(3) = v[k].head(3);
        wbcInterfaceData.baseVelocity[k].tail(3) = rpyDotTOtwist(q[0][3], q[0][4], q[0][5]) * v[k].segment(3, 3);//(yaw, pitch, roll)*(yaw, pitch, roll)


        // std::cout << "\nq:___\n" << q[k][3] << "," << q[k][4] << "," <<q[k][5] << "\n";

        // std::cout <<"Mat:\n" << rpyDotTOtwist(q[k][3], q[k][4], q[k][5]) << "\n";

        // std::cout << "angular vel\n" << wbcInterfaceData.baseVelocity[k].tail(3);

        
        // Base Acceleration
        pinocchio::computeCentroidalMapTimeVariation(model, data, q[k], v[k]); //the time derivative of the Centroidal Momentum Matrix
        vector_t hDot = vector_t::Zero(6);
        for(size_t j = 0; j < N_contactPoint; j++){
            // XYZ centroidal dynamics
            hDot.head(3) += mpcData.inputTrajectory_[k].segment(3*j, 3);
            Eigen::Matrix<double, 3, 1> f = mpcData.inputTrajectory_[k].segment(3*j, 3);
            Eigen::Matrix<double, 3, 1> r = wbcInterfaceData.swingFeetPosition[k][j] - q[k].head(3); //
            hDot.tail(3) += r.cross(f);
        }
        hDot[2] -= data.mass[0] * 9.81f;
        Eigen::Matrix<double, 6, 1> q_base_ddot;
        q_base_ddot = InverseAb[k] * (hDot - data.dAg * v[k] - Aj[k] * a[k].tail(numOfActuatedJoint));
        a[k].head(6) = q_base_ddot;
        // wbcInterfaceData.baseAcceleration[k].resize(6);
        // wbcInterfaceData.baseAcceleration[k].head(3) = q_base_ddot.head(3);
        // wbcInterfaceData.baseAcceleration[k].tail(3) = rpyDotTOtwist(q[k][3], q[k][4], q[k][5]) * q_base_ddot.tail(3) + rpyDotTOtwistDot(q[k][3], q[k][4], q[k][5], v[k][3], v[k][4], v[k][5]) * v[k].segment(3, 3);
        
        // std::cout << "\nangular acc\n" << wbcInterfaceData.baseAcceleration[k].tail(3);


        // Contact Point Velocity and Acceleration
        for(size_t j = 0; j < N_contactPoint; j++){
            matrix_t jacobianContactPoint = matrix_t::Zero(6, dofOfRobot);
            matrix_t jacobianDotContactPoint = matrix_t::Zero(6, dofOfRobot);
            pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings.contactNames3DoF[j]), pinocchio::LOCAL_WORLD_ALIGNED, jacobianContactPoint);   
            pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId(modelSettings.contactNames3DoF[j]), pinocchio::LOCAL_WORLD_ALIGNED, jacobianDotContactPoint);
            // Contact Point Velocity
            wbcInterfaceData.swingFeetVelocity[k][j] = (jacobianContactPoint * v[k]).head(3);
            // swingfootfile <<mpcData.timeTrajectory_[k] << "\t" << wbcInterfaceData.swingFeetVelocity[k][0][2] << "\n";

            // Contact Point Acceleration rdotdot = J*qdotdot + Jdot*qdot
            wbcInterfaceData.swingFeetAcceleration[k][j] = (jacobianContactPoint * a[k] + jacobianDotContactPoint * v[k]).head(3);
        } 

        //For LOCAL Swing
        pinocchio::computeJointJacobians(model, data, qTpl);
        pinocchio::updateFramePlacements(model, data);

        for(size_t j = 0; j < N_contactPoint; j++){
            wbcInterfaceData.swingFeetPosition[k][j].resize(3);
            wbcInterfaceData.swingFeetPosition[k][j] = data.oMf[model.getBodyId(modelSettings.contactNames3DoF[j])].translation();
        }

        // std::cout << "hdot:\n__________\n"<<hDot << "\n";
        // std::cout << "G:\n__________\n"<<model.gravity << "\n";
        // std::cout << "G981:\n__________\n"<<model.gravity981 << "\n";

    }
    // swingfootfile.close();
    // std::cout << "swingp0:\n__________\n"<<wbcInterfaceData.swingFeetPosition[0][0] << "\n";
    // std::cout << "swingp1:\n__________\n"<<wbcInterfaceData.swingFeetPosition[0][1] << "\n";
    // std::cout << "swingp2:\n__________\n"<<wbcInterfaceData.swingFeetPosition[0][2] << "\n";
    // std::cout << "swingp3:\n__________\n"<<wbcInterfaceData.swingFeetPosition[0][3] << "\n";

    // std::cout << "a:\n__________\n"<<a[5] << "\n";
    // std::cout << "mass:\n__________\n"<<data.mass[0] << "\n";
    // std::cout << "v:\n__________\n"<<v[5] << "\n";
    // std::cout << "q:\n__________\n"<<q[5] << "\n";
}


Eigen::Matrix<double, 4, 1> modeNumber2StanceLeg_WBC(const size_t& modeNumber){
    Eigen::Matrix<double, 4, 1> rst;
    switch (modeNumber)
    {
    case 0:  //lf lh rf rh
        rst << 0, 0, 0, 0;
        break;
    case 1:
        rst << 0, 0, 0, 1;
        break;
    case 2:
        rst << 0, 1, 0, 0;
        break;
    case 3:
        rst << 0, 1, 0, 1;
        break;
    case 4:
        rst << 0, 0, 1, 0;
        break;
    case 5:
        rst << 0, 0, 1, 1;
        break;
    case 6:
        rst << 0, 1, 1, 0;
        break;
    case 7:
        rst << 0, 1, 1, 1;
        break;
    case 8:
        rst << 1, 0, 0, 0;
        break;
    case 9:
        rst << 1, 0, 0, 1;
        break;
    case 10:
        rst << 1, 1, 0, 0;
        break;
    case 11:
        rst << 1, 1, 0, 1;
        break;
    case 12:
        rst << 1, 0, 1, 0;
        break;
    case 13:
        rst << 1, 0, 1, 1;
        break;
    case 14:
        rst << 1, 1, 1, 0;
        break;
    case 15:
        rst << 1, 1, 1, 1;
        break;
    
    default:
        rst << 1, 1, 1, 1;
        break;
    }
    return rst;
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