//
// Created by qiayuan on 2022/6/24.
// czworldy 2023/04/30
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "legged_controllers/LeggedController.h"

#include "geometry_msgs/PointStamped.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_jypro/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <ocs2_wbc_ros/SingleWbcRos.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  std::string wbcFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  controller_nh.getParam("/wbcFile", wbcFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  // robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
  //                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
  //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_unique<ocs2::wbc::SingleWbcRos>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), 
                                                        *eeKinematicsPtr_, wbcFile, nh);
  // wbc_->loadTasksSetting(taskFile, verbose);
  simpleMotion_ = std::make_unique<ocs2::wbc::SimpleMotion>(wbc_->getUserParam(), false);
  

  lf_foot_pub_ = nh.advertise<geometry_msgs::PointStamped>("/lf_foot_pos", 1);
  lh_foot_pub_ = nh.advertise<geometry_msgs::PointStamped>("/lh_foot_pos", 1);
  rf_foot_pub_ = nh.advertise<geometry_msgs::PointStamped>("/rf_foot_pos", 1);
  rh_foot_pub_ = nh.advertise<geometry_msgs::PointStamped>("/rh_foot_pos", 1);


  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;
  currentObservation_.state = leggedInterface_->getInitialState();

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
  ocs2::TargetTrajectories targetTrajectories(mpcMrtInterface_->getPolicy().timeTrajectory_,
                                              mpcMrtInterface_->getPolicy().stateTrajectory_,
                                              mpcMrtInterface_->getPolicy().inputTrajectory_);
  // optimizedInput = mpcMrtInterface_->getPolicy().inputTrajectory_.front(); // disable feedback MPC.
  if (optimizedInput.maxCoeff() > 1500.0 || optimizedInput.minCoeff() < -1500.0) {
    std::cout << "feedback optimizedInput: " << optimizedInput.transpose() << std::endl;
    optimizedInput = targetTrajectories.getDesiredInput(currentObservation_.time); // disable feedback MPC.
    ROS_WARN_STREAM("MPC input is too large, using the desired input instead.");
    std::cout << "get desired optimizedInput: " << optimizedInput.transpose() << std::endl;
  }
  // optimizedInput = targetTrajectories.getDesiredInput(currentObservation_.time); // disable feedback MPC.
  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec(), currentObservation_.time);
  wbcTimer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 50, 5, torque(j));
  }

  // Visualization
  // robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  // selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  currentObservation_.mode = plannedMode;
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

  // Publish mpc modeSchedule
  policyPublisher_.publish
    ((ros_msg_conversions::createModeScheduleMsg(mpcMrtInterface_->getPolicy().modeSchedule_)));
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  vector_t jointTorque(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  Eigen::Matrix<bool, 4, 1> contact_flag = {false, false, false, false}; //contact_flag lf lh rf rh for simplemotion
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTorque(i) = hybridJointHandles_[i].getEffort();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
    contact_flag[i] = contactHandles_[i].isContact();
  }
  contact_flag[1] = contactFlag[2];
  contact_flag[2] = contactFlag[1];
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateJointTorque(jointTorque);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  // currentObservation_.mode = stateEstimate_->getMode();

  vector_t qMeasured_(leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
  qMeasured_.head<3>() = measuredRbdState_.segment<3>(3);
  qMeasured_.segment<3>(3) = measuredRbdState_.head<3>();
  qMeasured_.tail(leggedInterface_->getCentroidalModelInfo().actuatedDofNum) 
                          = measuredRbdState_.segment(6, leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
  const auto& model = leggedInterface_->getPinocchioInterface().getModel();
  auto& data = leggedInterface_->getPinocchioInterface().getData();

  pinocchio::forwardKinematics(model, data, qMeasured_);
  pinocchio::updateFramePlacements(model, data);

  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  std::vector<vector3_t> posDesired = eeKinematicsPtr_->getPosition(vector_t());

  auto terrainInfo = simpleMotion_->TerrainEst(contact_flag, posDesired, quat.toRotationMatrix());

  terrainReceiverPtr_->setMpcTerrain(terrainInfo);

  const vector3_t& bodyPosition = currentObservation_.state.segment(6, 3);
  const vector3_t& bodyZyxEulerAngles = currentObservation_.state.segment(9, 3);
  Eigen::Matrix<scalar_t, 4, 4> _O_B_tfMatrix = Eigen::Matrix<scalar_t, 4, 4>::Identity();
  
  _O_B_tfMatrix.topLeftCorner(3, 3) = ocs2::getRotationMatrixFromZyxEulerAngles(bodyZyxEulerAngles);
  _O_B_tfMatrix.topRightCorner(3, 1) = bodyPosition;
  const auto _B_O_tfMatrix = _O_B_tfMatrix.inverse();

  std::vector<geometry_msgs::PointStamped> feet_pos;
  feet_pos.resize(4);
  for(size_t leg = 0; leg < 4; leg++){
    vector3_t posDesiredinBodyFrame = (_B_O_tfMatrix * posDesired[leg].homogeneous()).head(3);

    feet_pos[leg].point.x = posDesiredinBodyFrame.x();
    feet_pos[leg].point.y = posDesiredinBodyFrame.y();
    feet_pos[leg].point.z = posDesiredinBodyFrame.z();
  }
  lf_foot_pub_.publish(feet_pos[0]);
  rf_foot_pub_.publish(feet_pos[1]);
  lh_foot_pub_.publish(feet_pos[2]);
  rh_foot_pub_.publish(feet_pos[3]);
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_unique<ocs2::legged_robot::LeggedRobotInterface>(taskFile, urdfFile, referenceFile);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_unique<ocs2::MultipleShootingMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(), 
                                                     leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());
  // raisimConversions_ = std::make_shared<LeggedRobotRaisimConversions>(leggedInterface_->getPinocchioInterface(),
  //                             leggedInterface_->getCentroidalModelInfo(), leggedInterface_->getInitialState());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nodeHandle;
  // Gait receiver
  auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitReceiver>(
      nodeHandle, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::LeggedRobotRosReferenceManager>(robotName, leggedInterface_->getSwitchedModelReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // Terrain receiver
  // auto terrainReceiverPtr = std::make_shared<ocs2::legged_robot::TerrainReceiver>(nodeHandle,
  //     leggedInterface_->getSwitchedModelReferenceManagerPtr()->getTerrainEstDataPtr(), robotName);
  terrainReceiverPtr_ = std::make_shared<ocs2::legged_robot::TerrainPythonInterface>(
            leggedInterface_->getSwitchedModelReferenceManagerPtr()->getTerrainEstDataPtr());

  auto footPlacementPublisher = std::make_shared<ocs2::legged_robot::FootPlacementVisualizer>(nodeHandle, *leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());

  auto polygonReceiverPtr = std::make_shared<ocs2::legged_robot::LegEndEffectorsPolygonReceiver>
                            (nodeHandle, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getMpcPolygonArrayPtr(), 
                            leggedInterface_->getSwitchedModelReferenceManagerPtr()->getMpcNominalFeetholdsPtr(),
                            leggedInterface_->getSwitchedModelReferenceManagerPtr()->getMpcSwingHeightPtr(),
                            leggedInterface_->getSwitchedModelReferenceManagerPtr()->getMpcSwingMiddleTimePtr(),
                              robotName);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);  //for perRun
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);       //for preRun
  mpc_->getSolverPtr()->addSynchronizedModule(terrainReceiverPtr_);       //for preRun
  mpc_->getSolverPtr()->addSynchronizedModule(footPlacementPublisher);       //for preRun
  mpc_->getSolverPtr()->addSynchronizedModule(polygonReceiverPtr);
  observationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  policyPublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_activepolicy", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
