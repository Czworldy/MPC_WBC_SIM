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

#include <ocs2_mpc/MPC_DDP.h>
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
  LeggedRobotPyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile) {

    
    // Robot interface
    std::ifstream urdfStringFile(urdfFile);
    if (!urdfStringFile.is_open())
      throw std::runtime_error("urdfStringFile open failed. Aborting.");
    // std::string urdfString((std::istreambuf_iterator<char>(urdfStringFile)), std::istreambuf_iterator<char>());
    std::stringstream ss;
    ss << urdfStringFile.rdbuf();
    const std::string urdfString = ss.str();

    LeggedRobotInterface leggedRobotInterface(taskFile, libraryFolder, urdf::parseURDF(urdfString));

    // System dimensions
    // stateDim_ = leggedRobotInterface.getCentroidalModelInfo().stateDim;
    // inputDim_ = leggedRobotInterface.getCentroidalModelInfo().inputDim;

    // MPC
    std::unique_ptr<MPC_DDP> mpcPtr(new MPC_DDP(leggedRobotInterface.mpcSettings(), leggedRobotInterface.ddpSettings(),
                                                leggedRobotInterface.getRollout(), leggedRobotInterface.getOptimalControlProblem(),
                                                leggedRobotInterface.getInitializer()));
    
    auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitPythonInterface>(
            leggedRobotInterface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), 
            "/home/yjy/MPC_WBC_sim/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_jypro/config/command/gait.info", true);

    mpcPtr->getSolverPtr()->setReferenceManager(leggedRobotInterface.getReferenceManagerPtr());
    mpcPtr->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);       


    // Python interface
    PythonInterface::init(leggedRobotInterface, std::move(mpcPtr));
  }
};

}  // namespace ballbot
}  // namespace ocs2
