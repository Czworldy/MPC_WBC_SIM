#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  std::string str = "/home/yjy/.raisim/activation.raisim";
  std::cout << "binary path: " << str << std::endl;
  raisim::World::setActivationKey(str);

  raisim::World world;
  world.setTimeStep(0.001);

/// create raisim objects
  world.addGround(0, "gnd");

  auto robot = world.addArticulatedSystem("/home/yjy/JYPro/urdf/JYPro_ocs2.urdf");
    std::cout << "add Done" << std::endl;
  robot->setName("jypro");
  Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.0, -0.87, 1.78, -0.0, -0.87, 1.78, 0.0, -0.87, 1.78, -0.0, -0.87, 1.78;
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
  // robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(robot);

  for (int i=0; i<20000000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}