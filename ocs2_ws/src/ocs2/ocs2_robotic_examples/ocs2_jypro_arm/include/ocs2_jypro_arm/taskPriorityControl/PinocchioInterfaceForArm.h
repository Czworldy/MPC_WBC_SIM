#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <ArmSettings.h>

#include "common/Types.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <fstream>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace ocs2 {
namespace legged_robot {
namespace arm {

class PinocchioInterfaceForArm final {

    public:
        using Model = pinocchio::Model;
        using Data = pinocchio::Data;
        using JointModel = pinocchio::JointModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
        using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;

        explicit PinocchioInterfaceForArm(const ::urdf::ModelInterfaceSharedPtr& urdfTree, const std::vector<std::string>& jointNames);
        ~PinocchioInterfaceForArm(){};

        void gripperJacobiMatrix(const vector_t& jointStates, matrix_t& jacobi);
        void baseJacobiMatrix(const vector_t& jointStates, matrix_t& jacobi);
        void gripperPosVel(const vector_t& jointPos, const vector_t& jointVel, vector6_t& gripperPos, vector6_t& gripperVel);

        int nq; // dof

    private:
        scalar_t square(scalar_t a);

        vector3_t quaternionTOrpy(quaternion_t q);

        std::shared_ptr<const Model> robotModelPtr_;
        std::unique_ptr<Data> robotDataPtr_;
        std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr_;
        ArmSettings armSettings_;
        bool debug;

};

} // namespace arm
} // namespace legged_robot
} // namespace ocs2