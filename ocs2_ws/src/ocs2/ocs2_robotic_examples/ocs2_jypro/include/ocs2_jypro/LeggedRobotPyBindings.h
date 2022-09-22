/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_python_interface/PythonInterface.h>

#include "ocs2_jypro/LeggedRobotInterface.h"
#include "ocs2_jypro/gait/GaitPythonInterface.h"
// #include "ocs2_jypro/definitions.h"
#include <urdf_parser/urdf_parser.h>


#include <iostream>
#include <fstream>
#include <sstream>


namespace ocs2 {
namespace legged_robot {


class LeggedRobotPyBindings final : public PythonInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF of the robot. This is not used for ballbot.
   */
  LeggedRobotPyBindings(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {

    
    // // Robot interface
    // std::ifstream urdfStringFile(urdfFile);
    // if (!urdfStringFile.is_open())
    //   throw std::runtime_error("urdfStringFile open failed. Aborting.");
    // // std::string urdfString((std::istreambuf_iterator<char>(urdfStringFile)), std::istreambuf_iterator<char>());
    // std::stringstream ss;
    // ss << urdfStringFile.rdbuf();
    // const std::string urdfString = ss.str();

    leggedRobotInterfacePtr_.reset(new ocs2::legged_robot::LeggedRobotInterface(taskFile, urdfFile, referenceFile));

    // System dimensions
    // stateDim_ = leggedRobotInterface.getCentroidalModelInfo().stateDim;
    // inputDim_ = leggedRobotInterface.getCentroidalModelInfo().inputDim;

    // MPC
    std::unique_ptr<GaussNewtonDDP_MPC> mpcPtr(new GaussNewtonDDP_MPC(leggedRobotInterfacePtr_->mpcSettings(), leggedRobotInterfacePtr_->ddpSettings(),
                                                leggedRobotInterfacePtr_->getRollout(), leggedRobotInterfacePtr_->getOptimalControlProblem(),
                                                leggedRobotInterfacePtr_->getInitializer()));
    
    gaitReceiverPtr_.reset(new ocs2::legged_robot::GaitPythonInterface(
            leggedRobotInterfacePtr_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), 
            "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/gait.info", true));

    mpcPtr->getSolverPtr()->setReferenceManager(leggedRobotInterfacePtr_->getReferenceManagerPtr());
    mpcPtr->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr_);       

    auto initState = leggedRobotInterfacePtr_->getInitialState();
    const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(24);


    // Python interface
    PythonInterface::init(*leggedRobotInterfacePtr_, std::move(mpcPtr));
    setObservation(0.0, initState, zeroInput);
  }

  void setModule(const std::string& moduleName) override {
    std::cout << "setModule: " << moduleName << std::endl;
    gaitReceiverPtr_->setMpcModeSequence(moduleName);
  }

  vector_t getInitState() override {
    return leggedRobotInterfacePtr_->getInitialState();
  }

  int getStateDim()  {
    return leggedRobotInterfacePtr_->getCentroidalModelInfo().stateDim;
  }

  int getInputDim()  {
    return leggedRobotInterfacePtr_->getCentroidalModelInfo().inputDim;
  }

  
  // hold this interface to keep it alive, beacuse some classes reference its member
  // for example, FootPlacementPlanner's centroidalModelInfo_
  std::unique_ptr<LeggedRobotInterface> leggedRobotInterfacePtr_ = nullptr; 
  std::shared_ptr<ocs2::legged_robot::GaitPythonInterface> gaitReceiverPtr_ = nullptr;
};

}  // namespace ballbot
}  // namespace ocs2
