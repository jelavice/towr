/*
 * m545_model.h
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

#include "excavator_model/ExcavatorModel.hpp"
#include "excavator_model/Limits.hpp"
#include "loco_m545/common/typedefs.hpp"

namespace towr {

/**
 * \addtogroup Robots
 * @{
 */
class M545KinematicModelFull : public KinematicModel
{
 public:

  using MatrixXd = Eigen::MatrixXd;
  using JointLimitMap = std::map<std::string, std::map<std::string, double>>;

  //joint that are of interest for the planning
  enum joints
  {
    J_LF_HAA,
    J_LF_HFE,
    J_LF_STEER,
    J_RF_HAA,
    J_RF_HFE,
    J_RF_STEER,
    J_LH_HFE,
    J_LH_HAA,
    J_LH_STEER,
    J_RH_HFE,
    J_RH_HAA,
    J_RH_STEER,
    J_TURN,
    J_BOOM,
    J_DIPPER,
    J_TELE,
    J_EE_PITCH,
    NUM_JOINTS
  };

  M545KinematicModelFull(const std::string &urdfDescription, double dt);
  EEPos GetEEPositions(VectorXd jointAngles);
  MatrixXd GetEEJacobian(VectorXd jointAngles);

 private:

  void InitializeJointLimits();
  void CalculateJointLimitsforSpecificLeg(const excavator_model::Limits &limtis,
                                          loco_m545::RD::LimbEnum limb, unsigned int dof);

  void PrintJointLimits();

  excavator_model::ExcavatorModel model_;
  JointLimitMap joint_limits_;
  Eigen::Matrix<double, joints::NUM_JOINTS, 1> upperJointLimits_;
  Eigen::Matrix<double, joints::NUM_JOINTS, 1> lowerJointLimits_;

};

class M545DynamicModelFull : public SingleRigidBodyDynamics
{
 public:
  M545DynamicModelFull()
      : SingleRigidBodyDynamics(3700.0, 2485.29, 1640.58, 1600.99, 0.0, 0.0, 0.0, 4)
  {
  }
};
/** @}*/

} /* namespace towr */
