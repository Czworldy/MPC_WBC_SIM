#include <TaskPriorityControl.h>

namespace ocs2 {
namespace legged_robot {
namespace arm {

TaskPriorityControl::TaskPriorityControl() {
    // URDF Model -> Pinocchio Model
    std::string urdfPath = "/home/dqwang/MPC_WBC_arm/ocs2_ws/src/X20/urdf/ARM_ocs2.urdf";
    pinocchioInterfacePtr_.reset(new PinocchioInterfaceForArm(urdf::parseURDFFile(urdfPath), armSettings_.jointNames));
}

void TaskPriorityControl::TPcontrolLaw(const vector_t& jointSpacePosition, const GripperBaseVelocity& endEffectorVelocity, vector_t& jointSpaceVelocity) {
  pinocchioInterfacePtr_->gripperJacobiMatrix(jointSpacePosition, gripperJacobi_);
  pinocchioInterfacePtr_->baseJacobiMatrix(jointSpacePosition, baseJacobi_);
  taskPrioritySetUp(endEffectorVelocity);

  int dimStates(jointSpaceVelocity.size());
  jointSpaceVelocity = vector_t::Zero(dimStates);

  matrix_t jacobi_bar = jacobiVector_[1] * nullSpaceCal(jacobiVector_[0]);
  jointSpaceVelocity = pseudoInverse(jacobiVector_[0]) * velocityVector_[0] + pseudoInverse(jacobi_bar) * velocityVector_[1];
  // jointSpaceVelocity = pseudoInverse(jacobiVector_[0]) * velocityVector_[0] + nullSpaceCal(jacobiVector_[0]) * pseudoInverse(jacobiVector_[1]) * velocityVector_[1];
}

void TaskPriorityControl::taskPrioritySetUp(const GripperBaseVelocity& endEffectorVelocity) {
  
  jacobiTaskA_ = gripperJacobi_.block(0, 0, 6, 12); // Task 1: Gripper_xyzrpy
  jacobiTaskB_.resize(3, 12);
  jacobiTaskB_.topRows(2) = baseJacobi_.block(0, 0, 2, 12);    // Task 2: Base_xy yaw
  jacobiTaskB_.bottomRows(1) = baseJacobi_.block(3, 0, 1, 12);  

  velTaskA_.resize(6);
  velTaskB_.resize(3);

  velTaskA_ << endEffectorVelocity.gripper_x, endEffectorVelocity.gripper_y, endEffectorVelocity.gripper_z,
               endEffectorVelocity.gripper_yaw, endEffectorVelocity.gripper_pitch, endEffectorVelocity.gripper_roll;      
  velTaskB_ << endEffectorVelocity.base_x, endEffectorVelocity.base_y, endEffectorVelocity.base_yaw;   
  
  jacobiVector_.clear();
  velocityVector_.clear();

  jacobiVector_.resize(2);
  velocityVector_.resize(2);

  jacobiVector_[0] = jacobiTaskA_;
  jacobiVector_[1] = jacobiTaskB_;

  velocityVector_[0] = velTaskA_;
  velocityVector_[1] = velTaskB_;

  // jacobiVector_[0] = jacobiTaskA_;
  // jacobiVector_[1] = jacobiTaskB_;
  // jacobiVector_[2] = jacobiTaskC_;
  // jacobiVector_[3] = jacobiTaskD_;

  // velocityVector_[0] = velTaskA_;
  // velocityVector_[1] = velTaskB_;
  // velocityVector_[2] = velTaskC_;
  // velocityVector_[3] = velTaskD_;

  taskNum_ = 2;
}

void TaskPriorityControl::nullSpaceProjectorIteration(const matrix_t& jacobi, matrix_t& nullSpaceMat) {
  matrix_t nullSpaceMatTmp(nullSpaceMat);
  nullSpaceMat = nullSpaceMatTmp - pseudoInverse(jacobi*nullSpaceMatTmp) * jacobi * nullSpaceMatTmp;
}


matrix_t TaskPriorityControl::nullSpaceCal(const matrix_t& A) {
    matrix_t Anull;
    matrix_t inv, ident;
    size_t col;
    col = A.cols();
    ident = matrix_t::Identity(col,col);
	  inv = this -> pseudoInverse(A);
    Anull = ident - inv * A;

    return Anull;
}

matrix_t TaskPriorityControl::pseudoInverse(matrix_t const& matrix, scalar_t sigmaThreshold) {
  matrix_t invMatrix;

  if (  (1 == matrix.rows()) && (1 == matrix.cols()) ) {
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    return invMatrix;
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
      invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
      printf("sigular value is too small: %f\n",svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
  return invMatrix;
}

} // namespace arm
}
} // namespace ocs2