// pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
// WBC
#include "ocs2_wbc_ros/SingleWbcRos.h"
#include "ocs2_wbc/SimpleMotion/SimpleMotion.h"
// C++
#include <iostream>
#include <unistd.h>
#include <memory>
#include <chrono>
#include <thread>
// ROS
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
// #include "gazebo_msgs/LinkStates.h"   //yjy::可以不用LinkStates而使用ModelStates
// #include "gazebo_msgs/ModelStates.h"
// #include "ocs2_msgs/mpc_wbc_conversion.h"
// #include "ocs2_msgs/mpc_terrain.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ContactsState.h>
#include <fstream>
#include "pronto_msgs/QuadrupedStance.h"
#include "geometry_msgs/PointStamped.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_jypro/LeggedRobotPyBindings.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_jypro/common/ModelSettings.h>
#include <ocs2_jypro/gait/GaitTable.h>
#include <ocs2_jypro/gait/MotionPhaseDefinition.h>
#include <ocs2_jypro/BodyPositionEstimator/BodyPositionEstimator.h>
#include "ocs2_jypro/gait/GaitReceiver.h"
#include <ocs2_jypro/synchronized_module/LeggedRobotRosReferenceManager.h>
#include "ocs2_jypro/visualization/FootPlacementVisualizer.h"
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>


using namespace std;
using namespace ocs2;
using namespace wbc;

enum ControlFlag {
    kWaitForMsg = 0,
    kPDWaitForStanding = 1,
    kPDStandUpMotion = 2,
    kWBCBaseMotion = 3,
	kWBCMPC = 4,
	kSafeState = 5,
};
int robotState = 0;
bool isJointMsg(false);
bool isGazeboMsg(false);

bool isSetUp_PDWaitForStanding(false);
bool isSetUp_PDStandUpMotion(false);
bool isSetUp_SafeState(false);
bool isStandUp(false);
bool isSetUp_WBCBaseMotion(false);
LimbsPosVel jointStatesCur;

const scalar_t timePDWaitForStanding(2.0);
const scalar_t timePDStandUpMotion(2.0);
const scalar_t timeWBCBaseMotion(4.0);

const scalar_t haa_PDWaitForStanding(0);
const scalar_t hfe_PDWaitForStanding(-1.23);
const scalar_t kfe_PDWaitForStanding(2.79);

const scalar_t haa_PDStandUpMotion(0);
const scalar_t hfe_PDStandUpMotion(-0.95);
const scalar_t kfe_PDStandUpMotion(1.78);

std::atomic_bool controllerRunning_{}, mpcRunning_{};

Eigen::Matrix<bool, 4, 1> contact_flag_real = {false, false, false, false};

vector_t gazeboFeedbackJointPos, gazeboFeedbackJointVel; //lf rf lh rh

vector3_t basePosWorldCur;
vector3_t baseLinearVelWorldCur;
vector3_t baseLinearVelBodyCur;
Eigen::Quaterniond baseOriWorldCur;
vector3_t baseAngularVelWorldCur;
vector3_t baseAngularVelBodyCur;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void gazeboNavMsgsCallback(const nav_msgs::Odometry::ConstPtr& msg);
void contactStateCallback(const pronto_msgs::QuadrupedStance::ConstPtr& msg);
template <typename T>
inline T square(T a) {
  return a * a;
}

template <typename SCALAR_T>
inline Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

int main(int argc, char**argv) {
    //FOR ROS
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle nodeHandle;
    ros::Subscriber jointStatesSub, gazeboLinkStatesSub, mpcSub, contactStateSub;; 
    // ros::Subscriber lf_contactState, rf_contactState, lh_contactState, rh_contactState;

    ros::Publisher lf_haa_pub, lf_hfe_pub, lf_kfe_pub;
    ros::Publisher lh_haa_pub, lh_hfe_pub, lh_kfe_pub;
    ros::Publisher rf_haa_pub, rf_hfe_pub, rf_kfe_pub;
    ros::Publisher rh_haa_pub, rh_hfe_pub, rh_kfe_pub;
    ros::Publisher mpc_terrain_sync_input;
    ocs2_msgs::mpc_terrain mpc_terrain_sync_input_msg;

    ros::Publisher lf_foot_pub, lh_foot_pub, rf_foot_pub, rh_foot_pub;

    
    lf_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/lf_foot_pos", 1);
    lh_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/lh_foot_pos", 1);
    rf_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/rf_foot_pos", 1);
    rh_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/rh_foot_pos", 1);

    std::vector<vector3_t> feet_point_pos;

    // geometry_msgs::PointStamped lf_foot_pos, lh_foot_pos, rf_foot_pos, rh_foot_pos;
    std::vector<geometry_msgs::PointStamped> feet_pos;
    feet_pos.resize(4);

  
    std_msgs::Float64 lf_haa_tau, lf_hfe_tau, lf_kfe_tau;
    std_msgs::Float64 lh_haa_tau, lh_hfe_tau, lh_kfe_tau;
    std_msgs::Float64 rf_haa_tau, rf_hfe_tau, rf_kfe_tau;
    std_msgs::Float64 rh_haa_tau, rh_hfe_tau, rh_kfe_tau;

    ros::Rate rate(1000);

    const std::string wbcfilename = "/home/yjy/jy_control_test/include/PARAMETER/UserParameter_sdk_ws.info";
    const std::string taskfile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/mpc/task.info";
    const std::string referencefile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/targetTrajectories.info";
    const std::string urdffile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/X20/urdf/X20_rsm.urdf";
    const std::string gaitfile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/gait.info";
    const std::string robotName = "legged_robot";

    ocs2::legged_robot::ModelSettings modelSettings_;

    ros::Publisher observationPublisher = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
    auto interfacePtr = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(taskfile, urdffile, referencefile);
    // auto mpc = std::make_unique<ocs2::legged_robot::LeggedRobotPyBindings>(std::move(interfacePtr), gaitfile);
    auto mpc = std::make_shared<ocs2::MultipleShootingMpc>(interfacePtr->mpcSettings(), interfacePtr->sqpSettings(), 
                              interfacePtr->getOptimalControlProblem(), interfacePtr->getInitializer());
    auto rbdConversions = std::make_shared<CentroidalModelRbdConversions>(interfacePtr->getPinocchioInterface(),
                                                                        interfacePtr->getCentroidalModelInfo());
    auto raisimConversions = std::make_shared<LeggedRobotRaisimConversions>(interfacePtr->getPinocchioInterface(),
                                interfacePtr->getCentroidalModelInfo(), interfacePtr->getInitialState());
    // Gait receiver
    auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitReceiver>(
        nodeHandle, interfacePtr->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<ocs2::LeggedRobotRosReferenceManager>(robotName, interfacePtr->getSwitchedModelReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);

    // Terrain receiver
    // auto terrainReceiverPtr = std::make_shared<ocs2::legged_robot::TerrainReceiver>(nodeHandle,
    //     interfacePtr->getSwitchedModelReferenceManagerPtr()->getTerrainEstDataPtr(), robotName);
    auto terrainReceiverPtr = std::make_shared<ocs2::legged_robot::TerrainPythonInterface>(
              interfacePtr->getSwitchedModelReferenceManagerPtr()->getTerrainEstDataPtr());

    auto footPlacementPublisher = std::make_shared<ocs2::legged_robot::FootPlacementVisualizer>(nodeHandle, *interfacePtr->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());

    auto polygonReceiverPtr = std::make_shared<ocs2::legged_robot::LegEndEffectorsPolygonReceiver>
                              (nodeHandle, interfacePtr->getSwitchedModelReferenceManagerPtr()->getMpcPolygonArrayPtr(), 
                              interfacePtr->getSwitchedModelReferenceManagerPtr()->getMpcNominalFeetholdsPtr(),
                                robotName);
    mpc->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);  //for perRun
    mpc->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);       //for preRun
    mpc->getSolverPtr()->addSynchronizedModule(terrainReceiverPtr);       //for preRun
    mpc->getSolverPtr()->addSynchronizedModule(footPlacementPublisher);       //for preRun
    mpc->getSolverPtr()->addSynchronizedModule(polygonReceiverPtr);

    auto mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc);
    mpcMrtInterface_->initRollout(&interfacePtr->getRollout());

    controllerRunning_ = true;
    auto mpcThread_ = std::thread([&]() {
        while (controllerRunning_) {
            try {
                ocs2::executeAndSleep(
                        [&]() {
                            if (mpcRunning_) {
                                // mpcTimer_.startTimer();
                                mpcMrtInterface_->advanceMpc();
                                // mpcTimer_.endTimer();
                            }
                        }, 
                        100);
                        // qmInterface_->mpcSettings().mpcDesiredFrequency_);
            } catch (const std::exception& e) {
                controllerRunning_ = false;
                ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
            }
        }
    });
    ocs2::setThreadPriority(50, mpcThread_);

    ocs2::PinocchioInterface pinocchioInterface = interfacePtr->getPinocchioInterface();
    ocs2::CentroidalModelPinocchioMapping pinocchioMapping(interfacePtr->getCentroidalModelInfo());
    ocs2::PinocchioEndEffectorKinematics endEffectorKinematics(interfacePtr->getPinocchioInterface(), pinocchioMapping,
                                                        modelSettings_.contactNames3DoF);
    auto endEffectorKinematicsClonePtr = endEffectorKinematics.clone();
    auto wbc = std::make_shared<ocs2::wbc::SingleWbcRos>(interfacePtr->getPinocchioInterface(), interfacePtr->getCentroidalModelInfo(), 
                                                          endEffectorKinematics, wbcfilename, nodeHandle);
    auto simpleMotion = std::make_shared<ocs2::wbc::SimpleMotion>(wbc->getUserParam(), false);
  
    jointStatesSub = nodeHandle.subscribe("/X20/joint_states", 1, &jointStatesCallback);
    gazeboLinkStatesSub = nodeHandle.subscribe("/ground_truth/state", 1,&gazeboNavMsgsCallback);
    contactStateSub = nodeHandle.subscribe("/state_estimator_pronto/stance", 1, &contactStateCallback);
    // lf_contactState = nodeHandle.subscribe("/LF_contact", 1, &lf_bumper_callback); 
    // rf_contactState = nodeHandle.subscribe("/RF_contact", 1, &rf_bumper_callback); 
    // lh_contactState = nodeHandle.subscribe("/LH_contact", 1, &lh_bumper_callback); 
    // rh_contactState = nodeHandle.subscribe("/RH_contact", 1, &rh_bumper_callback); 

    lf_haa_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/LF_HAA_controller/command",1);
    lf_hfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/LF_HFE_controller/command",1);
    lf_kfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/LF_KFE_controller/command",1);
    lh_haa_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/LH_HAA_controller/command",1);
    lh_hfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/LH_HFE_controller/command",1);
    lh_kfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/LH_KFE_controller/command",1);
    rf_haa_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/RF_HAA_controller/command",1);
    rf_hfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/RF_HFE_controller/command",1);
    rf_kfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/RF_KFE_controller/command",1);
    rh_haa_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/RH_HAA_controller/command",1);
    rh_hfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/RH_HFE_controller/command",1);
    rh_kfe_pub = nodeHandle.advertise<std_msgs::Float64>("/X20/RH_KFE_controller/command",1);

    const int gvDim_ = 18, nJoints_ = 12;
    ocs2::vector_t initState = ocs2::vector_t::Zero(6 + gvDim_), gc_init_(19); // [Hcom, q_b, q_j]
    ocs2::vector_t initInput = ocs2::vector_t::Zero(12 + nJoints_); // [Force, qdot_j]
    gc_init_.segment(3, 4) << 1.0, 0.0, 0.0, 0.0;
    gc_init_.head(3) << interfacePtr->getInitialState().segment(6, 3);
    gc_init_.tail(12) = interfacePtr->getInitialState().tail(12);
    std::cout << "gc_init_ = " << gc_init_.transpose() << std::endl;

    initState.tail(gvDim_) << gc_init_.head(3), 0, 0, 0, gc_init_.tail(nJoints_);
    ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initState}, {initInput});
    SystemObservation currentObservation;
    currentObservation.state = initState;
    currentObservation.input = initInput;
    currentObservation.time = ros::Time::now().toSec();
    currentObservation.mode = ModeNumber::STANCE;

    LimbsCommand command;
    ros::AsyncSpinner spinner(6);
    spinner.start();
    while(nodeHandle.ok()){
        vector_t qMeasured(19), vMeasured(18);
        //ocs2 joint order [LF, LH, RF, RH]
        //gaebzo joint order   lf rf lh rh = raisim order
        qMeasured << basePosWorldCur, baseOriWorldCur.w(), baseOriWorldCur.x(), 
                                  baseOriWorldCur.y(), baseOriWorldCur.z(), gazeboFeedbackJointPos;
        // std::cout << "qMeasured: " << qMeasured.transpose() << std::endl;
        vMeasured << baseLinearVelWorldCur, baseAngularVelWorldCur, gazeboFeedbackJointVel;



        // simpleMotion->EstimatedStatesInput(estStatesOutput);
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
          simpleMotion->update(jointStatesCur);
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
          simpleMotion->update(jointStatesCur);
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
          simpleMotion->update(jointStatesCur);
		    	simpleMotion->PDMotionRun(command);
          currentObservation.time = ros::Time::now().toSec();
          mpcMrtInterface_->setCurrentObservation(currentObservation);
          mpcMrtInterface_->getReferenceManager().setTargetTrajectories(initTargetTrajectories);
          ROS_INFO_STREAM("\033[32m Waiting for the initial policy ... \033[0m");
          if (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
              mpcMrtInterface_->advanceMpc();
              // ros::WallRate(100).sleep();
          }
          else{
            ROS_INFO_STREAM("\033[32m Initial policy has been received. \033[0m");
            mpcRunning_ = true;
            robotState = kWBCMPC;
          }
		    	break;
		    }
		    case kWBCMPC: {
          const ocs2::vector_t& state = raisimConversions->raisimGenCoordGenVelToState(qMeasured, vMeasured); // [Hcom, q_b, q_j] //q_j order is fixed.
          const ocs2::vector_t& rbdState = raisimConversions->raisimGenCoordGenVelToRbdState(qMeasured, vMeasured);
          currentObservation.state = state;
          //contact_flag_real LF LH RF RH
          contact_flag_t stanceLegs = {contact_flag_real[0], contact_flag_real[2], 
                                        contact_flag_real[1], contact_flag_real[3]};// {LF, RF, LH, RH}
          currentObservation.mode = stanceLeg2ModeNumber(stanceLegs); 
          currentObservation.time = ros::Time::now().toSec();
          mpcMrtInterface_->setCurrentObservation(currentObservation);

          observationPublisher.publish(ros_msg_conversions::createObservationMsg(currentObservation));

          vector_t qMeasured_(interfacePtr->getCentroidalModelInfo().generalizedCoordinatesNum);
          qMeasured_.head<3>() = rbdState.segment<3>(3);
          qMeasured_.segment<3>(3) = rbdState.head<3>();
          qMeasured_.tail(interfacePtr->getCentroidalModelInfo().actuatedDofNum) 
                                  = rbdState.segment(6, interfacePtr->getCentroidalModelInfo().actuatedDofNum);
          const auto& model = pinocchioInterface.getModel();
          auto& data = pinocchioInterface.getData();

          pinocchio::forwardKinematics(model, data, qMeasured_);
          pinocchio::updateFramePlacements(model, data);

          endEffectorKinematicsClonePtr->setPinocchioInterface(pinocchioInterface);
          std::vector<vector3_t> posDesired = endEffectorKinematicsClonePtr->getPosition(vector_t());

          auto terrainInfo = simpleMotion->TerrainEst(contact_flag_real, posDesired, baseOriWorldCur.toRotationMatrix());

          terrainReceiverPtr->setMpcTerrain(terrainInfo);

          // Load the latest MPC policy
          mpcMrtInterface_->updatePolicy();

          // Evaluate the current policy
          vector_t optimizedState;
          vector_t optimizedInput;
          size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
          mpcMrtInterface_->evaluatePolicy(currentObservation.time, currentObservation.state, optimizedState, optimizedInput, plannedMode);
          // optimizedInput = mpcMrtInterface_->getPolicy().inputTrajectory_.front(); // disable feedback MPC.

          vector_t x = wbc->update(optimizedState, optimizedInput, rbdState, plannedMode, 0.001, currentObservation.time);

          // after solve the mpc problem, set target trajectory
          vector_t torque = x.tail(12); //ocs2 joint order [LF, LH, RF, RH]
          // Eigen::Map<vector3_t>(lf_pos.value) = q_j.head(3); 
          //TODO: add PD term
          Eigen::Map<vector3_t>(command.lf_tau.value) = torque.segment<3>(0);
          Eigen::Map<vector3_t>(command.lh_tau.value) = torque.segment<3>(3);
          Eigen::Map<vector3_t>(command.rf_tau.value) = torque.segment<3>(6);
          Eigen::Map<vector3_t>(command.rh_tau.value) = torque.segment<3>(9);
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

        lf_foot_pub.publish(feet_pos[0]);
        lh_foot_pub.publish(feet_pos[1]);
        rf_foot_pub.publish(feet_pos[2]);
        rh_foot_pub.publish(feet_pos[3]);

        // ros::spinOnce();

        // bool rate_bool = rate.sleep();
    }
    spinner.stop();

    return 0;
}

void gazeboNavMsgsCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    basePosWorldCur[0] = msg->pose.pose.position.x;
    basePosWorldCur[1] = msg->pose.pose.position.y;
    basePosWorldCur[2] = msg->pose.pose.position.z;

    baseLinearVelWorldCur[0] = msg->twist.twist.linear.x; 
    baseLinearVelWorldCur[1] = msg->twist.twist.linear.y; 
    baseLinearVelWorldCur[2] = msg->twist.twist.linear.z; 

    baseOriWorldCur.w() = msg->pose.pose.orientation.w;
    baseOriWorldCur.x() = msg->pose.pose.orientation.x;
    baseOriWorldCur.y() = msg->pose.pose.orientation.y;
    baseOriWorldCur.z() = msg->pose.pose.orientation.z;

    baseAngularVelWorldCur[0] = msg->twist.twist.angular.x;
    baseAngularVelWorldCur[1] = msg->twist.twist.angular.y;
    baseAngularVelWorldCur[2] = msg->twist.twist.angular.z;

    baseLinearVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseLinearVelWorldCur;

    baseAngularVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseAngularVelWorldCur; 

    isGazeboMsg = true;
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
    // observation.state = Eigen::Map<const Eigen::VectorXf>(state.data(), state.size()).cast<scalar_t>();
    // lf rf lh rh
    gazeboFeedbackJointPos = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
    gazeboFeedbackJointVel = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());
    isJointMsg = true;
}

void contactStateCallback(const pronto_msgs::QuadrupedStance::ConstPtr& msg){
    // LF
    contact_flag_real << false, false, false, false;
    if(msg->lf != 0)
        contact_flag_real[0] = true; // For Body Position Estimator ---- LF LH RF RH
                                             // For MPC Input Data ---- LF RF LH RH
    // lh
    if(msg->lh != 0)
        contact_flag_real[1] = true;
    if(msg->rf != 0)
        contact_flag_real[2] = true;
    if(msg->rh != 0)
        contact_flag_real[3] = true;
}