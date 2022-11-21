#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <string>

#include "ros/ros.h"
#include "time.h"
#include <ros/node_handle.h>
#include <ros/package.h>

#include "common/ModelSettings.h"
#include "common/ManipulatorModelInfo.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_msgs/mpc_observation.h>

using namespace ocs2;
using namespace legged_robot;

namespace {
std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
legged_robot::ModelSettings modelSettings;

ros::Publisher upperObservationPublisher;
ocs2_msgs::mpc_observation upperObservationMsg;

#define numOfActuatedJoint 18
#define numOfContactPoint 5
}  // namespace

void pseudoInverse(matrix_t const& matrix, double sigmaThreshold, matrix_t& invMatrix);


void leggedObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;

  ManipulatorModelType manipulatorType = static_cast<ManipulatorModelType>(4);
  upperObservationMsg.time = msg->time;
  upperObservationMsg.mode = 0;

  switch (manipulatorType) {
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      upperObservationMsg.state.value.resize(9);
      upperObservationMsg.input.value.resize(8);

      upperObservationMsg.state.value[0] = msg->state.value[6]; // x
      upperObservationMsg.state.value[1] = msg->state.value[7]; // y
      upperObservationMsg.state.value[2] = msg->state.value[9]; // yaw
      const auto lowerObservationStateDim = msg->state.value.size();
      const auto lowerObservationInputDim = msg->input.value.size();
      for (int i(0); i < 6; i++) {
        upperObservationMsg.state.value[i + 3] = msg->state.value[i + lowerObservationStateDim - 6];
      }

      // input
      const auto& model = pinocchioInterfacePtr->getModel();
      auto& data = pinocchioInterfacePtr->getData();
      vector_t leggedJointState(lowerObservationStateDim - 6);
      for (int i(0); i<leggedJointState.size(); i++) {
        leggedJointState[i] = msg->state.value[6 + i];
      }
      const auto& Ag = pinocchio::computeCentroidalMap(model, data, leggedJointState);
      matrix_t InverseAb;
      pseudoInverse(Ag.leftCols(6), 0.0001, InverseAb); // InverseAb
      matrix_t Aj = Ag.rightCols(numOfActuatedJoint); // Aj
      vector6_t hcom;
      vector_t actuatedJointState(numOfActuatedJoint);
      for (int i(0); i < 6; i++) {
        hcom[i] = msg->state.value[i];
      }
      for (int i(0); i < numOfActuatedJoint; i++) {
        actuatedJointState[i] = msg->input.value[3 * numOfContactPoint + i];
      }
      vector_t vBase = InverseAb * (hcom * data.mass[0] - Aj * actuatedJointState);

      upperObservationMsg.input.value[0] = sqrt(pow(vBase[0], 2) + pow(vBase[1], 2));
      upperObservationMsg.input.value[1] = vBase[3];
      for(int i(0); i < 6; i++) {
        upperObservationMsg.input.value[2 + i] = msg->input.value[lowerObservationInputDim - 6 + i];
      }

      vector_t observation_state(9);
      vector_t observation_input(8);
      for(int i(0); i<8; i++) {
        observation_input[i] = upperObservationMsg.input.value[i];
        observation_state[i] = upperObservationMsg.state.value[i];
      }
      observation_state[8] = upperObservationMsg.state.value[8];
      std::cout << "\nI GET THE MSG AND PUBLISH A NEW MSG!" << std::endl;
      std::cout << "observation_time: " << upperObservationMsg.time << std::endl;
      std::cout << "observation_state: \n" << observation_state << std::endl;
      std::cout << "observation_input: \n" << observation_input << std::endl;

      break;
    }
    case ManipulatorModelType::OmniBasedMobileManipulator: {
      break;
    }
  }
  
  upperObservationPublisher.publish(upperObservationMsg);
}

int main(int argc, char** argv) {
    std::vector<std::string> programArgs{};
    ::ros::removeROSArgs(argc, argv, programArgs);

    ros::init(argc, argv, "LowerLayerConversion");
    ros::NodeHandle nh;

    upperObservationPublisher = nh.advertise<ocs2_msgs::mpc_observation>("/mobile_manipulator_mpc_observation", 1);
    ros::Subscriber leggedObservationSubscriber;
    leggedObservationSubscriber = nh.subscribe("/legged_robot_with_arm_mpc_observation", 1, &leggedObservationCallback);

    // URDF Model -> Pinocchio Model
    std::string urdfPath = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/X20_ARM/urdf/X20_ARM_ocs2.urdf";
    pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfPath, modelSettings.jointNames)));

    std::cout << "START SPIN!" << std::endl;
    // while (ros::ok() && ros::master::check()) {
    //     ros::spinOnce();
    // }
    ros::spin();
    return 0;
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
