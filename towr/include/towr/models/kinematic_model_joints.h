/*
 * kinematic_model_joints.h
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */
#pragma once

#include "kinematic_model.h"

namespace towr {

class KinematicModelJoints : public KinematicModel
{
 public:

  /**
   * @brief Constructs a kinematic model of a robot with zero range of motion.
   * @param n_ee  The number of endeffectors of the robot.
   */
  KinematicModelJoints(const std::vector<int> &limbdofs, int n_ee)
      : KinematicModel(n_ee)
  {
    //todo add here all the joints and limb info
    num_dof_limbs_ = limbdofs;
  }

  virtual ~KinematicModelJoints() = default;

  int GetNumDof(int limbId) override
  {
    return num_dof_limbs_.at(limbId);
  }

 protected:

  std::vector<int> num_dof_limbs_;

};

} /* namespace towr */
