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

  //joint that are of interest for the planning
  enum Joints
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

  enum LimbStartIndex
  {
    LF = J_LF_HAA,
    RF = J_RF_HAA,
    LH = J_LH_HFE,
    RH = J_RH_HFE,
    BOOM = J_TURN
  };

  static constexpr unsigned int legDof = 3;
  static constexpr unsigned int boomDof = NUM_JOINTS - 4 * legDof;

  using MatrixXd = Eigen::MatrixXd;
  using JointLimitMap = std::unordered_map<std::string, std::unordered_map<std::string, double>>;
  using JointVector = Eigen::Matrix<double, Joints::NUM_JOINTS, 1>;
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using EEJac = std::vector<SparseMatrix>;

  M545KinematicModelFull(const std::string &urdfDescription, double dt);

  // these are in the base frame
  const EEPos &GetEEPositions(const VectorXd &jointAngles);

  // this is in the world frame
  const EEPos &GetEEOrientation(const VectorXd &jointAngles);

  const EEJac &GetTranslationalJacobians(const VectorXd &jointAngles);
  const JointVector &GetLowerLimits();
  const JointVector &GetUpperLimits();

 private:

  //update base stuff and joints
  void UpdateModel(const VectorXd &jointAngles, const Vector3d &ypr );

  void InitializeJointLimits();
  void CalculateJointLimitsforSpecificLimb(const excavator_model::Limits &limtis,
                                           loco_m545::RD::LimbEnum limb, unsigned int dof);
  //update joints
  void UpdateModel(const VectorXd &jointAngles);

  void PrintJointLimits();
  void UpdateSpecificLimb(loco_m545::RD::LimbEnum limb, const VectorXd &jointAngles,
                          unsigned int dof);

  void ExtractOptimizedJoints(const MatrixXd &bigJacobian, loco_m545::RD::LimbEnum limb,
                              LimbStartIndex limbStartIndex,
                              unsigned int dof);

  excavator_model::ExcavatorModel model_;
  JointLimitMap joint_limits_;
  JointVector upper_joint_limits_;
  JointVector lower_joint_limits_;
  EEPos ee_pos_;
  EEJac ee_trans_jac_;

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
