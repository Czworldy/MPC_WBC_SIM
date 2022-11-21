#include <PinocchioInterfaceForArm.h>

namespace ocs2 {
namespace legged_robot {
namespace arm {

PinocchioInterfaceForArm::PinocchioInterfaceForArm(const ::urdf::ModelInterfaceSharedPtr& urdfTree, const std::vector<std::string>& jointNames) {
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    // remove extraneous joints from urdf
    for (joint_pair_t& jointPair : urdfTree->joints_) {
        if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) == jointNames.end()) {
            jointPair.second->type = urdf::Joint::FIXED;
        }
    }

    // add 3 DoF for the floating base
    pinocchio::JointModelComposite  jointComposite(2);
    jointComposite.addJoint(pinocchio::JointModelTranslation());
    jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdfTree, jointComposite, model);

    robotModelPtr_ = std::make_shared<const Model>(model);
    robotDataPtr_ = std::unique_ptr<Data>(new Data(*robotModelPtr_));
    if (urdfTree) {
        urdfModelPtr_ = std::make_shared<const ::urdf::ModelInterface>(*urdfTree);
    }
    nq = model.nq;
    debug = true;
}

void PinocchioInterfaceForArm::gripperJacobiMatrix(const vector_t& jointStates, matrix_t& jacobi){

    const auto& model = *robotModelPtr_;
    auto data = *robotDataPtr_;
    
    jacobi.resize(6, model.nq);
    pinocchio::computeJointJacobians(model, data, jointStates);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(armSettings_.endEffectorNames6DoF[0]), pinocchio::LOCAL_WORLD_ALIGNED, jacobi);   

    matrix_t jacobiTmp(jacobi);
 
    jacobi.block(3,0,1,model.nq) = jacobiTmp.block(5,0,1,model.nq);
    jacobi.block(5,0,1,model.nq) = jacobiTmp.block(3,0,1,model.nq);
}

void PinocchioInterfaceForArm::baseJacobiMatrix(const vector_t& jointStates, matrix_t& jacobi){

    const auto& model = *robotModelPtr_;
    auto data = *robotDataPtr_;
    
    jacobi.resize(6, model.nq);
    pinocchio::computeJointJacobians(model, data, jointStates);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(armSettings_.endEffectorNames6DoF[1]), pinocchio::LOCAL_WORLD_ALIGNED, jacobi); 

    matrix_t jacobiTmp(jacobi);
 
    jacobi.block(3,0,1,model.nq) = jacobiTmp.block(5,0,1,model.nq);
    jacobi.block(5,0,1,model.nq) = jacobiTmp.block(3,0,1,model.nq);

}

void PinocchioInterfaceForArm::gripperPosVel(const vector_t& jointPos, const vector_t& jointVel, vector6_t& gripperPos, vector6_t& gripperVel) {
    const auto& model = *robotModelPtr_;
    auto data = *robotDataPtr_;
    pinocchio::forwardKinematics(model, data, jointPos);
    pinocchio::updateFramePlacement(model, data, model.getBodyId(armSettings_.endEffectorNames6DoF[0]));
    gripperPos.head(3) =  data.oMf[model.getBodyId(armSettings_.endEffectorNames6DoF[0])].translation();
    quaternion_t gripperQuaternion(data.oMf[model.getBodyId(armSettings_.endEffectorNames6DoF[0])].rotation());
    gripperPos.tail(3) = quaternionTOrpy(gripperQuaternion); // yaw, pitch, roll

    // TODO gripperVel 
}

scalar_t PinocchioInterfaceForArm::square(scalar_t a) {
  return a * a;
}

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), returns
 * angles in (yaw-pitch-roll).
 */
vector3_t PinocchioInterfaceForArm::quaternionTOrpy(quaternion_t q){
  vector3_t rpy;
  scalar_t as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  rpy(0) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                 square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  rpy(1) = std::asin(as);
  rpy(2) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                 square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return rpy; // yaw, pitch, roll
 }






} // namespace arm
} // namespace legged_robot
} // namespace ocs2