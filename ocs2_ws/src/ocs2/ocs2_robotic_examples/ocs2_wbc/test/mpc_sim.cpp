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

#include <raisim/OgreVis.hpp>
// #include "raisimVis/raisimKeyboardCallback.hpp"
// #include "raisimVis/raisimBasicImguiPanel.hpp"

#include "ocs2_wbc/raisimVis/anymal/anymal_imgui_render_callback.hpp"
#include "ocs2_wbc/raisimVis/anymal/gaitLogger.hpp"
#include "ocs2_wbc/raisimVis/anymal/jointSpeedTorqueLogger.hpp"
#include "ocs2_wbc/raisimVis/anymal/rewardLogger.hpp"
#include "ocs2_wbc/raisimVis/anymal/videoLogger.hpp"
#include "ocs2_wbc/raisimVis/anymal/frameVisualizer.hpp"
#include "ocs2_wbc/SingleWbc.h"
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
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>
#include <signal.h>
#include <atomic>

#include <ros/ros.h>
#include <angles/angles.h>

using namespace raisim;
using namespace ocs2;
using namespace legged_robot;

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
    void clear() {
        value[0] = 0.0;
        value[1] = 0.0;
        value[2] = 0.0;
    }
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
    void clear(){
        lf_pos.clear();
        rf_pos.clear();
        lh_pos.clear();
        rh_pos.clear();
        lf_vel.clear();
        rf_vel.clear();
        lh_vel.clear();
        rh_vel.clear();
    }
}	LimbsPosVel;

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
    LimbsContacts contact;
    LimbsPosVel jointStates;

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
void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3,-3,-0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(10);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.03, 0.6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);

  /// skybox
  Ogre::Quaternion quat;
  quat.FromAngleAxis(Ogre::Radian(M_PI_2), {1., 0, 0});
  vis->getSceneManager()->setSkyBox(true,
                                    "Examples/StormySkyBox",
                                    500,
                                    true,
                                    quat);
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
  const std::string taskfile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/mpc/task.info";
  const std::string referencefile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/targetTrajectories.info";
  const std::string urdffile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/X20/urdf/X20_rsm.urdf";
  const std::string gaitfile = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/gait.info";

  observationPublisher = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
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

//   ocs2::PinocchioInterface pinocchioInterface(ocs2::centroidal_model::createPinocchioInterface(urdffile, modelSettings_.jointNames));
  ocs2::CentroidalModelPinocchioMapping pinocchioMapping(interfacePtr->getCentroidalModelInfo());
  ocs2::PinocchioEndEffectorKinematics endEffectorKinematics(interfacePtr->getPinocchioInterface(), pinocchioMapping,
                                                       modelSettings_.contactNames3DoF);
  auto wbc = std::make_unique<ocs2::wbc::SingleWbc>(interfacePtr->getPinocchioInterface(), interfacePtr->getCentroidalModelInfo(), endEffectorKinematics, wbcfilename);
  

  auto world = std::make_unique<raisim::World>();
  world->setTimeStep(0.001);
  world->setERP(0, 0);

  world->setMaterialPairProp("steel","rubber",0.8, 0.4, 0.001);
  world->setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001);
  world->setMaterialPairProp("steel","rubber",0.8, 0.4, 0.001);
  world->setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001);

  auto heightMap_ = world->addHeightMap("/home/yjy/jy_control_test/SlopeTerrain.txt",0, 0, "steel");

  heightMap_->setAppearance("soil2");
  heightMap_->setName("gnd");

  auto vis = raisim::OgreVis::get();

  /// gui
  anymal_gui::init({anymal_gui::video::init(vis->getResourceDir()),
                      anymal_gui::joint_speed_and_torque::init(100),
                      anymal_gui::gait::init(100),
                      // anymal_gui::reward::init({"commandTracking", "torque"}),
                      anymal_gui::frame::init()});

  /// these method must be called before initApp
  vis->setWorld(world.get());
  vis->setWindowSize(1800, 1200);
  vis->setImguiSetupCallback(raisim::anymal_gui::imguiSetupCallback);
  vis->setImguiRenderCallback(raisim::anymal_gui::anymalImguiRenderCallBack);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);

  /// starts visualizer thread
  vis->initApp();
  vis->createGraphicalObject(heightMap_, "gnd", "checkerboard_green");


  auto robot = world->addArticulatedSystem(urdffile);
  robot->setName("X20");
  robot->getCollisionBody("LF_SHANK/0").setMaterial("rubber");
  robot->getCollisionBody("LH_SHANK/0").setMaterial("rubber");
  robot->getCollisionBody("RF_SHANK/0").setMaterial("rubber");
  robot->getCollisionBody("RH_SHANK/0").setMaterial("rubber");

  auto robotVisual_ = vis->createGraphicalObject(robot, "X20");

  vis->setDesiredFPS(45.);
  vis->select(robotVisual_->at(0), false);
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);

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

  ocs2::QuaternionToRPY yawTotalCounter;
  yawTotalCounter.reset();
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
  const double simulation_dt_ = 0.001;
  const int visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);
  uint64_t resetMpcTargetCounter = 1;
  uint64_t visualizationCounter_ = 0;
  Eigen::VectorXd command_out(18);
  uint64_t sim_loop = 0;

  auto randomGenerateMpcTargetTrajtory = [&](const ocs2::vector_t& currentState) {

    ocs2::vector_t currentPose = currentState.segment(6, 6);

    // yawCommand_ = commandDist_(generator_);
    // xCommand_   = 1.5*commandDist_(generator_) + 0.2;  //add bias
    // yCommand_   = commandDist_(generator_);

    ocs2::scalar_t lastYawCommand_ = 0;
    ocs2::scalar_t lastXCommand_ = 0;
    ocs2::scalar_t lastYCommand_ = 0;

    double yawCommand_ = 0.;
    double xCommand_   = 0.5;
    double yCommand_   = 0.;
    
    // absLimiter(yawCommand_, 0.4);
    // absLimiter(xCommand_, 1.0);
    // absLimiter(yCommand_, 0.2);

    filter(yawCommand_, lastYawCommand_, 0.2);
    filter(xCommand_, lastXCommand_, 0.5);
    filter(yCommand_, lastYCommand_, 0.5);

    const ocs2::vector_t targetPose = [&]() {
      ocs2::vector_t target(6);
      // base p_x, p_y are relative to current state
      target(0) = currentPose(0) + xCommand_;
      target(1) = currentPose(1) + yCommand_;
      // base z relative to the default height
      target(2) = currentPose(2);
      // theta_z relative to current
      target(3) = currentPose(3) + yawCommand_;
      // theta_y, theta_x
      target(4) = 0;
      target(5) = 0;
      return target;
    }();

    // filter_->Process(commandDist_(generator_));
    std::cout << "currentPose: " << currentPose.transpose() << std::endl;
    std::cout << "targetPose: " << targetPose.transpose() << std::endl;

    // desired state trajectory
    ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(currentState.size()));
    stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose, gc_init_.tail(nJoints_);
    stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose, gc_init_.tail(nJoints_);

    // desired input trajectory (just right dimensions, they are not used)
   const ocs2::vector_array_t inputTrajectory(2, ocs2::vector_t::Zero(12 + nJoints_));

    ocs2::TargetTrajectories initTargetTrajectories({0.0, 1.0}, stateTrajectory, inputTrajectory);
    // mpc->setTargetTrajectories(initTargetTrajectories);

  };
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
    Eigen::Vector4i contact_flag_real = {0, 0, 0, 0};
    for(auto& contact: robot->getContacts()){
      if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
      if (LFfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd")
              estStatesOutput.contact.lf = 1.;
      }
      if (LHfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd")
              estStatesOutput.contact.lh = 1.;

      }
      if (RFfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd")
              estStatesOutput.contact.rf = 1.;
      }
      if (RHfootIndex == contact.getlocalBodyIndex()){
          if(world->getObject(contact.getPairObjectIndex())->getName() == "gnd")
              estStatesOutput.contact.rh = 1.;
      }
    }
    contact_flag_real[0] = (int) estStatesOutput.contact.lf;
    contact_flag_real[1] = (int) estStatesOutput.contact.lh;
    contact_flag_real[2] = (int) estStatesOutput.contact.rf;
    contact_flag_real[3] = (int) estStatesOutput.contact.rh;
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

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation.time, currentObservation.state, optimizedState, optimizedInput, plannedMode);

    // std::cout << "optimizedState: " << optimizedState.transpose() << std::endl;
    // std::cout << "optimizedInput: " << optimizedInput.transpose() << std::endl;
    // std::cout << "plannedMode: " << plannedMode << std::endl;
    // std::cout << "currentObservation.time: " << currentObservation.time << std::endl;

    vector_t x = wbc->update(optimizedState, optimizedInput, rbdState, plannedMode, 0.001, currentObservation.time);
    // std::cout << "wbc update: " << x.rows() << std::endl; //rows = 42
    // after solve the mpc problem, set target trajectory
    vector_t torque = x.tail(12);
    // std::cout << "torque: " << torque.transpose() << std::endl;
    if (resetMpcTargetCounter % 100 == 0 && resetMpcTargetCounter > 10 && setTarget) {
      
        randomGenerateMpcTargetTrajtory(state);
    }
    resetMpcTargetCounter++;


    command_out.tail(12) << torque.head(3), torque.segment(6, 3), torque.segment(3, 3), torque.tail(3);

    robot->setGeneralizedForce(command_out);
    // robot->setPdTarget(optimizedState

    world->integrate();
    // estimateState();


    if(visualizationCounter_ % visDecimation == 0) {
      OgreVis::get()->renderOneFrame();
    }
    ++visualizationCounter_;
    ++sim_loop;

    if(sim_loop % 5 == 0){
    // if(1){
      /// torque, speed and contact state
      // std::cout << "robotControlState: " << robotState << std::endl;
      Eigen::VectorXd jointTorque = robot->getGeneralizedForce().e().tail(12);
      Eigen::VectorXd jointSpeed = robot->getGeneralizedVelocity().e().tail(12);
      // anymal_gui::joint_speed_and_torque::push_back(sim_loop*world->getTimeStep(), jointSpeed, jointTorque);
      // anymal_gui::gait::push_back(contact_flag_real);
    }
    
    if(app_stopped){
        break;
        controllerRunning_ = false;
    }
  }

  return 0;
}