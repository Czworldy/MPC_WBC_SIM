// pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <stdlib.h>
#include <set>
#include <random>

// #include <raisim/OgreVis.hpp>
// #include "raisimVis/raisimKeyboardCallback.hpp"
// #include "raisimVis/raisimBasicImguiPanel.hpp"

// #include "ocs2_wbc/raisimVis/anymal/anymal_imgui_render_callback.hpp"
// #include "ocs2_wbc/raisimVis/anymal/gaitLogger.hpp"
// #include "ocs2_wbc/raisimVis/anymal/jointSpeedTorqueLogger.hpp"
// #include "ocs2_wbc/raisimVis/anymal/rewardLogger.hpp"
// #include "ocs2_wbc/raisimVis/anymal/videoLogger.hpp"
// #include "ocs2_wbc/raisimVis/anymal/frameVisualizer.hpp"
#include "ocs2_wbc_ros/SingleWbcRos.h"
#include "ocs2_wbc/SimpleMotion/SimpleMotion.h"
// #include "raisim_test.hpp"

// #include "MAIN/simpleMotion.h"
// #include "MAIN/wbc_definitions.h"
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
#include "ocs2_jypro/visualization/LeggedRobotVisualizer.h"
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>

#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>
#include <signal.h>
#include <atomic>

#include "ocs2_wbc_ros/RandomHeightMapGenerator.hpp"
#include <ros/ros.h>
#include <angles/angles.h>

#include "raisim/RaisimServer.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>


using namespace raisim;
using namespace ocs2;
using namespace legged_robot;


struct EstimatorOutput {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    EstimatorOutput(){ clear(); }
    float time_stamp;
    Eigen::Matrix<double, 3, 1> base_pos_world;
    Eigen::Matrix<double, 3, 1> base_linear_vel_world;
    Eigen::Matrix<double, 3, 1> base_linear_vel_body;
    Eigen::Quaterniond base_orientation_world;
    Eigen::Matrix<double, 3, 1> base_angular_vel_world;
    Eigen::Matrix<double, 3, 1> base_angular_vel_body;
    ocs2::wbc::LimbsContacts contact;
    ocs2::wbc::LimbsPosVel jointStates;

    Eigen::Matrix<double, 3, 1> frame_c_rpy_in_world;
    Eigen::Quaterniond frame_c_quat_in_world;
    Eigen::Matrix<double, 3, 1> frame_c_xyz_in_world;

    ocs2::legged_robot::TerrainEstData terrainEstData;

    void clear(){
        time_stamp = 0;
        base_pos_world.setZero();
        base_linear_vel_world.setZero();
        base_linear_vel_body.setZero();
        base_orientation_world.setIdentity();
        base_angular_vel_world.setZero();
        base_angular_vel_body.setZero();
        contact.lf = 0;
        contact.rf = 0;
        contact.lh = 0;
        contact.rh = 0;
        jointStates.clear();
        frame_c_rpy_in_world.setZero();
        frame_c_quat_in_world.setIdentity();
        frame_c_xyz_in_world.setZero();
        terrainEstData.terrainQuat.setIdentity();
        terrainEstData.terrainParams.setZero();
    }
};

EstimatorOutput estStatesOutput;

enum ControlFlag {
    kWaitForMsg = 0,
    kPDWaitForStanding = 1,
    kPDStandUpMotion = 2,
    kWBCBaseMotion = 3,
	kWBCMPC = 4,
	kSafeState = 5,
};

template <typename T>
T square(T a) {
    return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

const double timePDWaitForStanding(1.0);
const double timePDStandUpMotion(2.0);
const double timeWBCBaseMotion(4.0);

const double haa_PDWaitForStanding(0);
const double hfe_PDWaitForStanding(-1.23);
const double kfe_PDWaitForStanding(2.79);

const double haa_PDStandUpMotion(-0.04);
const double hfe_PDStandUpMotion(-0.676);
const double kfe_PDStandUpMotion(1.317);

const double xBase(0.0);
const double yBase(0.0);
const double zBase(0.0);
const double rollBase(0.);
const double pitchBase(0.);
const double yawBase(0.);

const double simulation_dt_ = 0.001;

bool isMPC(false);
bool isMPCMsgUpdate(false);

bool app_stopped = false;

std::atomic_bool controllerRunning_{}, mpcRunning_{};

ocs2::legged_robot::ModelSettings modelSettings_;

void sigint_handler(int sig){
	if(sig == SIGINT){
		// ctrl+c退出时执行的代码
		std::cout << "ctrl+c pressed!" << std::endl;
		app_stopped = true;
	}
}
void filter(ocs2::scalar_t& input, ocs2::scalar_t& lastOutput, ocs2::scalar_t alpha){
    lastOutput = alpha * input + (1 - alpha) * lastOutput;
    input = lastOutput;
}


int main(int argc, char* argv[]) {
  using vector12_t = Eigen::Matrix<ocs2::scalar_t, 12, 1>;
  using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
  using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;
  signal(SIGINT, sigint_handler);
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  std::string str = "/home/yjy/.raisim/activation.raisim";
  std::cout << "binary path: " << binaryPath.getString() << std::endl;
  raisim::World::setActivationKey(str);

  ros::init(argc, argv, "ocs2_wbc_test");
  ros::NodeHandle nodeHandle;
  const std::string robotName = "legged_robot";

  ros::Publisher observationPublisher;

  const std::string wbcfilename = "/home/yjy/jy_control_test/include/PARAMETER/UserParameter_sdk_ws.info";
  std::string taskfile      ;//= "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/mpc/task.info";
  std::string referencefile ;//= "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/targetTrajectories.info";
  std::string urdffile      ;//= "/home/yjy/MPC_WBC_sim/ocs2_ws/src/X20/urdf/X20_rsm.urdf";
  std::string gaitfile      ;//= "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/gait.info";

  nodeHandle.getParam("/gaitCommandFile", gaitfile);
  nodeHandle.getParam("/urdfFile", urdffile);
  nodeHandle.getParam("/taskFile", taskfile);
  nodeHandle.getParam("/referenceFile", referencefile);

  observationPublisher = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);

  auto lf_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/lf_foot_pos", 1);
  auto lh_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/lh_foot_pos", 1);
  auto rf_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/rf_foot_pos", 1);
  auto rh_foot_pub = nodeHandle.advertise<geometry_msgs::PointStamped>("/rh_foot_pos", 1);

  auto interfacePtr = std::make_unique<ocs2::legged_robot::LeggedRobotInterface>(taskfile, urdffile, referencefile);
  // auto mpc = std::make_unique<ocs2::legged_robot::LeggedRobotPyBindings>(std::move(interfacePtr), gaitfile);
  auto mpc = std::make_unique<ocs2::MultipleShootingMpc>(interfacePtr->mpcSettings(), interfacePtr->sqpSettings(), 
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
                            interfacePtr->getSwitchedModelReferenceManagerPtr()->getMpcSwingHeightPtr(),
                            interfacePtr->getSwitchedModelReferenceManagerPtr()->getMpcSwingMiddleTimePtr(),
                              robotName);
  mpc->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);  //for perRun
  mpc->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);       //for preRun
  mpc->getSolverPtr()->addSynchronizedModule(terrainReceiverPtr);       //for preRun
  mpc->getSolverPtr()->addSynchronizedModule(footPlacementPublisher);       //for preRun
  mpc->getSolverPtr()->addSynchronizedModule(polygonReceiverPtr);
  std::cout << "mpc done!\n";
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
  auto wbc = std::make_unique<ocs2::wbc::SingleWbcRos>(interfacePtr->getPinocchioInterface(), interfacePtr->getCentroidalModelInfo(), 
                                                        endEffectorKinematics, wbcfilename, nodeHandle);
  auto simpleMotion = std::make_unique<ocs2::wbc::SimpleMotion>(wbc->getUserParam(), false);
  auto robotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(interfacePtr->getPinocchioInterface(),
                                                             interfacePtr->getCentroidalModelInfo(), endEffectorKinematics, nodeHandle);

  

  auto world = std::make_unique<raisim::World>();
  world->setTimeStep(simulation_dt_);
  world->setERP(0, 0);

  world->setMaterialPairProp("steel","rubber",0.8, 0.4, 0.001);
  world->setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001);
  world->setMaterialPairProp("steel","rubber",0.8, 0.4, 0.001);
  world->setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001);

  raisim::RandomHeightMapGenerator hmGenerator;
  std::mt19937 gen(0);  

  // auto heightMap_ = world->addHeightMap("/home/yjy/jy_control_test/terrain.txt",0, 0, "steel");
  // auto heightMap_ = hmGenerator.generateTerrain(world.get(), raisim::RandomHeightMapGenerator::GroundType::HEIGHT_MAP_DISCRETE, 1, false, gen);
  auto ground = world->addGround();
  ground->setName("gnd");

  //map publisher
  std::unique_ptr<ocs2::RaisimHeightmapRosConverter> heightmapPub;
  heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
  auto mapPublishThread_ = std::thread([&]() {
      ros::Rate rate(1);
      while(ros::ok() && ros::master::check()) {
      //  heightmapPub->publishGridmap(*heightMap_, "odom");
       rate.sleep();
      }
  });

  // heightMap_->setAppearance("soil2");
  // heightMap_->setName("gnd");
  raisim::Mat<3, 3> inertia;
  inertia.setIdentity();
  const raisim::Vec<3> com = {0, 0, 0};
//   auto map = world->addMesh("/home/yjy/Documents/ICRA2023/meshes/part/map_easy.obj", 1.0, inertia, com, 1.0,"default");
  double map_scale = 1.0;
  nodeHandle.getParam("/map_scale", map_scale);
  auto map = world->addMesh("/home/yjy/Downloads/map_fix.obj", 1.0, inertia, com, map_scale, "default");
  map->setName("terrain");
  map->setBodyType(raisim::BodyType::STATIC);
  map->setOrientation( 0.0, 0.0, std::sqrt(2)/2.0, std::sqrt(2)/2.0);
  double init_x = 0;
  double init_y = 0;
  nodeHandle.getParam("/map_init_x", init_x);
  nodeHandle.getParam("/map_init_y", init_y);
  map->setPosition(init_x, init_y, 0.0);

  
  auto robot = world->addArticulatedSystem(urdffile);

  robot->setName("X20");
  // robot->getCollisionBody("LF_SHANK/0").setMaterial("rubber");
  // robot->getCollisionBody("LH_SHANK/0").setMaterial("rubber");
  // robot->getCollisionBody("RF_SHANK/0").setMaterial("rubber");
  // robot->getCollisionBody("RH_SHANK/0").setMaterial("rubber");

      /// get robot data
  int gcDim_ = robot->getGeneralizedCoordinateDim();
  int gvDim_ = robot->getDOF();
  int nJoints_ = gvDim_ - 6;

  /// initialize containers
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_;
  gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
  gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);

  /// this is nominal configuration of anymal
  gc_init_ << 0.0, 0.0, 0.44, 1.0, 0.0, 0.0, 0.0, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584;
  gc_init_.tail(12) = interfacePtr->getInitialState().tail(12);
  // gc_init_[2] = heightMap_->getHeight(0, 0) + 0.44;

  /// set pd gains
  Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
  jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(300.0);
  jointDgain.setZero(); jointDgain.tail(12) << 8, 6, 6, 8, 6, 6, 8, 6, 6, 8, 6, 6;
  robot->setState(gc_init_, gv_init_);
  robot->setGeneralizedCoordinate(gc_init_);
  robot->setGeneralizedVelocity(gv_init_);
  robot->setPdGains(jointPgain, jointDgain);
  robot->setPdTarget(gc_init_, gv_init_);
  robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE); //FORCE_AND_TORQUE  PD_PLUS_FEEDFORWARD_TORQUE

    //Note: ocs2::vector_t is different from vector_t
  ocs2::vector_t state = ocs2::vector_t::Zero(6 + gvDim_); // [Hcom, q_b, q_j]
  ocs2::vector_t input = ocs2::vector_t::Zero(12 + nJoints_); // [Force, qdot_j]
  // Eigen::Quaterniond initQuaternion(gc_init_(3), gc_init_(4), gc_init_(5), gc_init_(6)); // To be checked
  // auto initRPY = quaternionTOrpy(initQuaternion).reverse();

//   ocs2::QuaternionToRPY yawTotalCounter;
//   yawTotalCounter.reset();
  state.tail(gvDim_) << gc_init_.head(3), 0, 0, 0, gc_init_.tail(nJoints_);
  // state.tail(12) << -0.007, -0.84, 1.584, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584, -0.007, -0.84, 1.584;
  // std::cout << "initRPY: " << initRPY.transpose() << std::endl;
  ocs2::TargetTrajectories initTargetTrajectories({0.0}, {state}, {input});
  SystemObservation currentObservation;
  currentObservation.state = state;
  currentObservation.input = input;
  currentObservation.time = 0.0;
  currentObservation.mode = ModeNumber::STANCE;

  mpcMrtInterface_->setCurrentObservation(currentObservation);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(initTargetTrajectories);
  ROS_INFO_STREAM("\033[32m Waiting for the initial policy ... \033[0m");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
      mpcMrtInterface_->advanceMpc();
      ros::WallRate(100).sleep();
  }
  ROS_INFO_STREAM("\033[32m Initial policy has been received. \033[0m");
  mpcRunning_ = true;

  raisim::Vec<3> basePosWorldCur;
  raisim::Vec<3> baseLinearVelWorldCur;
  raisim::Vec<3> baseAngularVelWorldCur;
  raisim::Mat<3,3> baseOriWorldCur;

  int robotState = kWBCMPC; // skip standing process
  bool isSetUp_PDWaitForStanding(false);
  bool isSetUp_PDStandUpMotion(false);
  bool isSetUp_SafeState(false);
  bool isStandUp(false);
  bool isSetUp_WBCBaseMotion(false);

  bool isSafe(true);
  const double desired_fps_ = 45.0;
  const int visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);
  uint64_t resetMpcTargetCounter = 1;
  uint64_t visualizationCounter_ = 0;
  Eigen::VectorXd command_out(18);
  uint64_t sim_loop = 0;
Eigen::Matrix<bool, 4, 1> contact_flag_real = {false, false, false, false};

  auto estimateState = [&]() {
    estStatesOutput.time_stamp = sim_loop*world->getTimeStep();
    robot->getPosition(robot->getBodyIdx("base"), basePosWorldCur);
    raisim::VecDyn Q = robot->getGeneralizedCoordinate();
    raisim::VecDyn Qdot = robot->getGeneralizedVelocity();
    robot->getOrientation(robot->getBodyIdx("base"), baseOriWorldCur); // checked
    robot->getAngularVelocity(robot->getBodyIdx("base"), baseAngularVelWorldCur);
    baseLinearVelWorldCur = Qdot.e().topRows(3);

    size_t LFfootIndex = robot->getBodyIdx("LF_SHANK");
    size_t LHfootIndex = robot->getBodyIdx("LH_SHANK");
    size_t RFfootIndex = robot->getBodyIdx("RF_SHANK");
    size_t RHfootIndex = robot->getBodyIdx("RH_SHANK");
    /// for all contacts on the robot, check ...
    estStatesOutput.contact.lf = 0;
    estStatesOutput.contact.lh = 0;
    estStatesOutput.contact.rf = 0;
    estStatesOutput.contact.rh = 0;
    // Eigen::Vector4i contact_flag_real = {0, 0, 0, 0};
    for(auto& contact: robot->getContacts()){
      if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
      if (LFfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd" || world->getObject(contact.getPairObjectIndex())->getName() == "terrain")
              estStatesOutput.contact.lf = 1.;
      }
      if (LHfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd" || world->getObject(contact.getPairObjectIndex())->getName() == "terrain")
              estStatesOutput.contact.lh = 1.;

      }
      if (RFfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd" || world->getObject(contact.getPairObjectIndex())->getName() == "terrain")
              estStatesOutput.contact.rf = 1.;
      }
      if (RHfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd" || world->getObject(contact.getPairObjectIndex())->getName() == "terrain")
              estStatesOutput.contact.rh = 1.;
      }
    }
    contact_flag_real[0] = (bool) estStatesOutput.contact.lf;
    contact_flag_real[1] = (bool) estStatesOutput.contact.lh;
    contact_flag_real[2] = (bool) estStatesOutput.contact.rf;
    contact_flag_real[3] = (bool) estStatesOutput.contact.rh;
    // auto& feetcaontact = estStatesOutput.contact;
    // std::cout << "estStatesOutput.caontact: " << feetcaontact.lf << " " << feetcaontact.lh << " " << feetcaontact.rf << " " << feetcaontact.rh << std::endl;

    estStatesOutput.base_pos_world = basePosWorldCur.e();
    //Use Qdot as baseliner velocity in world frame. if this one =0 will casue shake in Z axis. 
    estStatesOutput.base_linear_vel_world = baseLinearVelWorldCur.e(); // checked
    estStatesOutput.base_linear_vel_body = baseOriWorldCur.e().transpose() * baseLinearVelWorldCur.e();
    estStatesOutput.base_orientation_world = baseOriWorldCur.e();
    estStatesOutput.base_angular_vel_world = baseAngularVelWorldCur.e();
    estStatesOutput.base_angular_vel_body = baseOriWorldCur.e().transpose() * baseAngularVelWorldCur.e();

    // raisim::quatToEulerVec() //TODO: checked this function
    Eigen::Vector3d baseRpyWorldCur = quatToZyx(estStatesOutput.base_orientation_world).reverse();
    // estStatesOutput.frame_c_rpy_in_world << 0, 0, baseRpyWorldCur[2];
    // estStatesOutput.frame_c_quat_in_world = rpyTOquaternion(0., 0., baseRpyWorldCur[2]);
    estStatesOutput.frame_c_rpy_in_world = baseRpyWorldCur;
    estStatesOutput.frame_c_quat_in_world = baseOriWorldCur.e(); //yjy：先试试都转

    estStatesOutput.frame_c_xyz_in_world = basePosWorldCur.e();

    for(int i(0); i < 3; i++) {
      estStatesOutput.jointStates.lf_pos.value[i] = Q[i+7];
      estStatesOutput.jointStates.rf_pos.value[i] = Q[i+10];
      estStatesOutput.jointStates.lh_pos.value[i] = Q[i+13];
      estStatesOutput.jointStates.rh_pos.value[i] = Q[i+16];

      estStatesOutput.jointStates.lf_vel.value[i] = Qdot[i+6];
      estStatesOutput.jointStates.rf_vel.value[i] = Qdot[i+9];
      estStatesOutput.jointStates.lh_vel.value[i] = Qdot[i+12];
      estStatesOutput.jointStates.rh_vel.value[i] = Qdot[i+15];	
    }

    // simpleMotion->EstimatedStatesInput(estStatesOutput);
    // estStatesOutput.terrainEstData = simpleMotion->TerrainEst(contact_flag_real);
  };
  bool isMpcReady(false), isMpcFirstSolved(false);

  size_t generalizedCoordinatesNum = interfacePtr->getCentroidalModelInfo().generalizedCoordinatesNum;

  raisim::RaisimServer server(world.get());
//   auto scans = server.addInstancedVisuals("scan points",
//                                           raisim::Shape::Box,
//                                           {0.01, 0.01, 0.01},
//                                           {1,0,0,1},
//                                           {0,1,0,1});
  int scanSize1 = 50;
  int scanSize2 = 100;

//   scans->resize(scanSize1*scanSize2);
  server.launchServer();
  std::vector<geometry_msgs::PointStamped> feet_pos;
  feet_pos.resize(4);
  ros::Rate rate_(1000);
  bool tfPublished = false;
  
  auto scanThread_ = std::thread([&]() {
    auto cloudPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("/point_cloud2",1);
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_(tfBuffer_);

      while (nodeHandle.ok()) {
          try {
              ocs2::executeAndSleep(
                [&]() {
                    if(!tfPublished)
                        return;
                    raisim::Vec<3> lidarPos; raisim::Mat<3,3> lidarOri;
                    robot->getFramePosition("lidar_joint", lidarPos);
                    robot->getFrameOrientation("lidar_joint", lidarOri);
                    std::string errorMsg;
                    ros::Time timeStamp = ros::Time(0);  // Use Time(0) to get the latest transform.
                    Eigen::Isometry3d transformation;
                    if (tfBuffer_.canTransform("lidar_link", "odom", timeStamp, &errorMsg)) {
                        geometry_msgs::TransformStamped transformStamped;
                        try {
                            transformStamped = tfBuffer_.lookupTransform("lidar_link", "odom", timeStamp);
                        } catch (tf2::TransformException& ex) {
                            ROS_ERROR("[ConvexPlaneExtractionROS] %s", ex.what());
                            transformation =  Eigen::Isometry3d::Identity();
                        }

                        // Extract translation.
                        transformation.translation().x() = transformStamped.transform.translation.x;
                        transformation.translation().y() = transformStamped.transform.translation.y;
                        transformation.translation().z() = transformStamped.transform.translation.z;

                        // Extract rotation.
                        Eigen::Quaterniond rotationQuaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                                                transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
                        transformation.linear() = rotationQuaternion.toRotationMatrix();
                    } else {
                        ROS_ERROR_STREAM("[ConvexPlaneExtractionROS] errorMsg" << errorMsg);
                        return;
                    }

                    sensor_msgs::PointCloud2 scanMsg;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                    for(int i=0; i<scanSize1; i++) {
                        for (int j = 0; j < scanSize2; j++) {
                            const double yaw = j * M_PI / scanSize2 * 0.6 - 0.3 * M_PI;
                            double pitch = -(i * 0.6/scanSize1) + 0.4;
                            const double normInv = 1. / sqrt(pitch * pitch + 1);
                            Eigen::Vector3d direction = {cos(yaw) * normInv, sin(yaw) * normInv, -pitch * normInv};
                            Eigen::Vector3d rayDirection;
                            rayDirection = lidarOri.e() * direction;
                            auto &col = world->rayTest(lidarPos.e(), rayDirection, 10);
                            if (col.size() > 0) {
                                // scans->setPosition(i * scanSize2 + j, col[0].getPosition());
                                float length = (col[0].getPosition() - lidarPos.e()).norm();
                                // scans->setColorWeight(i * scanSize2 + j, std::min(length/15.f, 1.0f));
                                pcl::PointXYZ p(col[0].getPosition()[0], col[0].getPosition()[1], col[0].getPosition()[2]);
                                cloud->points.push_back(p);
                            }
                            // else
                            //     scans->setPosition(i*scanSize2+j, {0, 0, 100});
                        }
                    }
                    // pcl::toROSMsg(*cloud, scanMsg);
                    // std::cout << "TF: " << transformation.matrix() << "\n";

                    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation.matrix().cast<float>());
                    pcl::toROSMsg(*transformed_cloud, scanMsg);
                    scanMsg.header.frame_id = "lidar_link"; 
                    scanMsg.header.stamp = ros::Time::now();
                    cloudPub.publish(scanMsg);

                }, 
                10);
          } catch (const std::exception& e) {
              controllerRunning_ = false;
              ROS_ERROR_STREAM("[Scan thread] Error : " << e.what());
          }
      }
  });
  while (nodeHandle.ok()) { 
    ros::spinOnce();
    estimateState();
    static bool setTarget = false;

    ocs2::vector_t state = raisimConversions->raisimGenCoordGenVelToState(robot->getGeneralizedCoordinate().e(), 
                                          robot->getGeneralizedVelocity().e()); // [Hcom, q_b, q_j] //q_j order is fixed.
    ocs2::vector_t rbdState = raisimConversions->raisimGenCoordGenVelToRbdState(robot->getGeneralizedCoordinate().e(), 
                                          robot->getGeneralizedVelocity().e());
    currentObservation.state = state;
    contact_flag_t stanceLegs = {(bool) estStatesOutput.contact.lf, (bool) estStatesOutput.contact.rf, 
                                  (bool) estStatesOutput.contact.lh, (bool) estStatesOutput.contact.rh};// {LF, RF, LH, RH}
    currentObservation.mode = stanceLeg2ModeNumber(stanceLegs); 
    currentObservation.time = sim_loop*world->getTimeStep();
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
    //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    std::vector<vector3_t> posDesired = endEffectorKinematicsClonePtr->getPosition(vector_t());
    const vector3_t& bodyPosition = state.segment(6, 3);
    const vector3_t& bodyZyxEulerAngles = state.segment(9, 3);
    Eigen::Matrix<scalar_t, 4, 4> _O_B_tfMatrix = Eigen::Matrix<scalar_t, 4, 4>::Identity();
    
    _O_B_tfMatrix.topLeftCorner(3, 3) = ocs2::getRotationMatrixFromZyxEulerAngles(bodyZyxEulerAngles);
    _O_B_tfMatrix.topRightCorner(3, 1) = bodyPosition;
    const auto _B_O_tfMatrix = _O_B_tfMatrix.inverse();
    for(size_t leg = 0; leg < 4; leg++){
     vector3_t posDesiredinBodyFrame = (_B_O_tfMatrix * posDesired[leg].homogeneous()).head(3);
 
     feet_pos[leg].point.x = posDesiredinBodyFrame.x();
     feet_pos[leg].point.y = posDesiredinBodyFrame.y();
     feet_pos[leg].point.z = posDesiredinBodyFrame.z();
    }

    auto terrainInfo = simpleMotion->TerrainEst(contact_flag_real, posDesired, baseOriWorldCur.e());

    terrainReceiverPtr->setMpcTerrain(terrainInfo);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation.time, currentObservation.state, optimizedState, optimizedInput, plannedMode);
    ocs2::TargetTrajectories targetTrajectories(mpcMrtInterface_->getPolicy().timeTrajectory_,
                                                mpcMrtInterface_->getPolicy().stateTrajectory_,
                                                mpcMrtInterface_->getPolicy().inputTrajectory_);
    // optimizedInput = mpcMrtInterface_->getPolicy().inputTrajectory_.front(); // disable feedback MPC.
    optimizedInput = targetTrajectories.getDesiredInput(currentObservation.time); // disable feedback MPC.
    // std::cout << "optimizedState: " << optimizedState.transpose() << std::endl;
    // std::cout << "optimizedInput: " << optimizedInput.transpose() << std::endl;
    // std::cout << "plannedMode: " << plannedMode << std::endl;
    // std::cout << "currentObservation.time: " << currentObservation.time << std::endl;

    vector_t x = wbc->update(optimizedState, optimizedInput, rbdState, plannedMode, simulation_dt_, currentObservation.time);
    // std::cout << "wbc update: " << x.rows() << std::endl; //rows = 42
    // after solve the mpc problem, set target trajectory
    vector_t torque = x.tail(12);
    // std::cout << "torque: " << torque.transpose() << std::endl;

    // if (resetMpcTargetCounter % 100 == 0 && resetMpcTargetCounter > 10 && setTarget) {
      
    //     randomGenerateMpcTargetTrajtory(state);
    // }
    // resetMpcTargetCounter++;

    command_out.tail(12) << torque.head(3), torque.segment(6, 3), torque.segment(3, 3), torque.tail(3);

    robot->setGeneralizedForce(command_out);
    // robot->setPdTarget(optimizedState

    // world->integrate();
    // estimateState();
    robotVisualizer_->update(currentObservation, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
    tfPublished = true;

    lf_foot_pub.publish(feet_pos[0]);
    rf_foot_pub.publish(feet_pos[1]);
    lh_foot_pub.publish(feet_pos[2]);
    rh_foot_pub.publish(feet_pos[3]);


    // if(visualizationCounter_ % visDecimation == 0) {
    //   // OgreVis::get()->renderOneFrame();
    // }
    // ++visualizationCounter_;
    ++sim_loop;



    // if(sim_loop % 5 == 0){
    // // if(1){
    //   /// torque, speed and contact state
    //   // std::cout << "robotControlState: " << robotState << std::endl;
    //   Eigen::VectorXd jointTorque = robot->getGeneralizedForce().e().tail(12);
    //   Eigen::VectorXd jointSpeed = robot->getGeneralizedVelocity().e().tail(12);
    //   // anymal_gui::joint_speed_and_torque::push_back(sim_loop*world->getTimeStep(), jointSpeed, jointTorque);
    //   // anymal_gui::gait::push_back(contact_flag_real);
    // }
    
    if(app_stopped){
        break;
        controllerRunning_ = false;
    }

    server.integrateWorldThreadSafe();
    rate_.sleep();
  }

  return 0;
}