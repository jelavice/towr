/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

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

#ifndef TOWR_MODELS_ROBOT_MODEL_H_
#define TOWR_MODELS_ROBOT_MODEL_H_

#include <map>
#include <string>

#include <towr/models/dynamic_model.h>
#include <towr/models/kinematic_model.h>

namespace towr {

/**
 * @defgroup Robots
 * @brief The kinematic and dynamic model of the robot.
 *
 * These models contain all the robot specific quantities in this problem.
 *
 * ###Further Reading:
 *  * RobotModel
 *
 * Folder: \ref include/towr/models.
 */

/**
 * @brief Holds pointers to the robot specific kinematics and dynamics.
 *
 * @ingroup RobotModels
 */
struct RobotModel {
  /**
   * @brief Examples robots for which kinematic and dynamic models exist.
   */
  enum Robot { Monoped, Biped, Hyq, Anymal, m545, ROBOT_COUNT };


  RobotModel() = default;
  RobotModel(Robot robot);

  KinematicModel::Ptr kinematic_model_;
  DynamicModel::Ptr   dynamic_model_;
};


const static std::map<RobotModel::Robot, std::string> robot_names =
{
  {RobotModel::Monoped, "Monoped"},
  {RobotModel::Biped,   "Biped"},
  {RobotModel::Hyq,     "Hyq"},
  {RobotModel::Anymal,  "Anymal"},
  {RobotModel::m545, "m545"}
};

} /* namespace towr */

#endif /* TOWR_MODELS_ROBOT_MODEL_H_ */
