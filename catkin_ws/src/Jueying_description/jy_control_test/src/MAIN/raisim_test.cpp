#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  std::string str = "/home/yjy/.raisim/activation.raisim";
  std::cout << "binary path: " << str << std::endl;
  raisim::World::setActivationKey(str);

  raisim::World world;
  world.setTimeStep(0.002);

/// create raisim objects
  auto ground = world.addGround();

  auto robot = world.addArticulatedSystem("/home/yjy/github/raisimLib/build/examples/rsc/anymal/urdf/anymal.urdf");
    std::cout << "add Done" << std::endl;
  robot->setName("jypro");
  Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
  jointNominalConfig.setZero(); jointVelocityTarget.setZero();

  std::cout << "sime of coordinate: " << robot->getGeneralizedCoordinateDim() << std::endl;
  std::cout << "sime of DOF: " << robot->getDOF() << std::endl;
//  raisim::Vec<4> quat; quat = {0, 0.0499792, 0, 0.9987503}; quat/= quat.norm();
//   gc.segment<7>(0) << 0, 0, 0.197, 1, 0, 0, 0;
  

  robot->setGeneralizedCoordinate(jointNominalConfig);
  robot->setGeneralizedVelocity(jointVelocityTarget);
//   robot->setGeneralizedForce({0, 0, 0, 0, 0, 0, 30, 30, 30, 30});
  robot->setIntegrationScheme(raisim::ArticulatedSystem::IntegrationScheme::SEMI_IMPLICIT);
  robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(robot);

  for (int i=0; i<20000000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(10000));
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}