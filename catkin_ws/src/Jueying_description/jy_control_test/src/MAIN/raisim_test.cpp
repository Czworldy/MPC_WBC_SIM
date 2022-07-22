#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "simpleMotion.h"
#include "utility.h"
#include "ros/ros.h"


std::shared_ptr<SimpleMotion> simpleMotion;
EstimatorOutput estStatesOutput;

enum ControlFlag {
    kWaitForMsg = 0,
    kPDWaitForStanding = 1,
    kPDStandUpMotion = 2,
    kWBCBaseMotion = 3,
	kWBCMPC = 4,
	kSafeState = 5,
};

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

conversionData mpcData;

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    std::string str = "/home/yjy/.raisim/activation.raisim";
    std::cout << "binary path: " << str << std::endl;
    ros::init(argc, argv, "raisim_test_node");
    ros::NodeHandle nh;
    simpleMotion.reset(new SimpleMotion(false));
    raisim::World::setActivationKey(str);

    raisim::World world;
    world.setTimeStep(0.001);

    /// create raisim objects

    world.addGround(0, "gnd");

    auto robot = world.addArticulatedSystem("/home/yjy/JYPro/urdf/JYPro_ocs2.urdf");
    std::cout << "add Done" << std::endl;
    robot->setName("jypro");
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    jointNominalConfig << 0, 0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, -0.87, 1.78, -0.0, -0.87, 1.78, 0.0, -0.87, 1.78, -0.0, -0.87, 1.78;
    jointVelocityTarget.setZero();

    std::cout << "sime of coordinate: " << robot->getGeneralizedCoordinateDim() << std::endl;
    std::cout << "sime of DOF: " << robot->getDOF() << std::endl;
    //  raisim::Vec<4> quat; quat = {0, 0.0499792, 0, 0.9987503}; quat/= quat.norm();
    //   gc.segment<7>(0) << 0, 0, 0.197, 1, 0, 0, 0;

    Eigen::VectorXd jointPgain(robot->getDOF()), jointDgain(robot->getDOF());
    jointPgain.tail(12).setConstant(100.0);
    jointDgain.tail(12).setConstant(1.0);

    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedVelocity(jointVelocityTarget);
    robot->setPdGains(jointPgain, jointDgain);
    robot->setPdTarget(jointNominalConfig, jointVelocityTarget);
    robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    // robot->setExternalForce(robot->getBodyIdx("base"),raisim::Vec<3>{0, 0, -9.8});

    // robot->printOutBodyNamesInOrder();
    // robot->printOutFrameNamesInOrder();
    raisim::Vec<3> basePosWorldCur;
    raisim::Vec<3> baseLinearVelWorldCur;
    raisim::Vec<3> baseAngularVelWorldCur;
    raisim::Mat<3,3> baseOriWorldCur;

    LimbsCommand command;  

    int robotState = 0;
    bool isSetUp_PDWaitForStanding(false);
    bool isSetUp_PDStandUpMotion(false);
    bool isSetUp_SafeState(false);
    bool isStandUp(false);
    bool isSetUp_WBCBaseMotion(false);

    bool isMPC(false);
    bool isMPCMsgUpdate(false);
    bool isJointMsg(false);
    bool isGazeboMsg(false);

    bool isSafe(true);

    ros::Rate rate(400);
    Eigen::VectorXd command_out(18);
    while (nh.ok()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1500));

        // estStatesOutput.time_stamp = ros::Time::now().toSec();
        robot->getPosition(robot->getBodyIdx("base"), basePosWorldCur);
        raisim::VecDyn Q = robot->getGeneralizedCoordinate();
        raisim::VecDyn Qdot = robot->getGeneralizedVelocity();
        robot->getOrientation(robot->getBodyIdx("base"), baseOriWorldCur);
        robot->getAngularVelocity(robot->getBodyIdx("base"), baseAngularVelWorldCur);
        baseLinearVelWorldCur = Qdot.e().topRows(3);


        estStatesOutput.base_pos_world = basePosWorldCur.e();
        // estStatesOutput.base_linear_vel_world = Qdot.e().topRows(3); //No INPUT
        estStatesOutput.base_linear_vel_world = baseLinearVelWorldCur.e(); // checked


        estStatesOutput.base_linear_vel_body = baseOriWorldCur.e().transpose() * baseLinearVelWorldCur.e();
        estStatesOutput.base_orientation_world = baseOriWorldCur.e();
        estStatesOutput.base_angular_vel_world = baseAngularVelWorldCur.e();
        estStatesOutput.base_angular_vel_body = baseOriWorldCur.e().transpose() * baseAngularVelWorldCur.e();

        Eigen::Vector3d baseRpyWorldCur = quaternionTOrpy(estStatesOutput.base_orientation_world);
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

        simpleMotion->EstimatedStatesInput(estStatesOutput);
        static int count = 0;
        std::cout << "robotState: " << robotState << std::endl;
        switch (robotState) {
            case kWaitForMsg: {
            //  std::this_thread::sleep_for(std::chrono::microseconds(1000));
                command_out.setZero();
                // count++;
                // if(count > 1000)
		    	    robotState = kPDWaitForStanding;

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
                    count++;
                    if(count > 300)
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
                std::cout << "WBCRun!\n";
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
		    		simpleMotion->UpdateMPCMsg(&mpcData, estStatesOutput.time_stamp);
                    simpleMotion->UpdateControlFrame(estStatesOutput);
		    		isMPCMsgUpdate = false;
		    	}
                // simpleMotion->TerrainEst(contact_flag_real);
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


        command_out.head(6) << 0, 0, 0, 0, 0, 0;

        command_out[6] = command.lf_tau.value[0];
        command_out[7] = command.lf_tau.value[1];
        command_out[8] = command.lf_tau.value[2];
        command_out[9] = command.rf_tau.value[0];
        command_out[10] = command.rf_tau.value[1];
        command_out[11] = command.rf_tau.value[2];
        command_out[12] = command.lh_tau.value[0];
        command_out[13] = command.lh_tau.value[1];
        command_out[14] = command.lh_tau.value[2];
        command_out[15] = command.rh_tau.value[0];
        command_out[16] = command.rh_tau.value[1];
        command_out[17] = command.rh_tau.value[2];

        robot->setGeneralizedForce(command_out);

        server.integrateWorldThreadSafe();
        // rate.sleep();

    }

    server.killServer();
}