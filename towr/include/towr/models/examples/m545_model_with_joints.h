/*
 * m545_model_with_joints.h
 *
 *  Created on: Aug 30, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/models/kinematic_model_with_joints.h>
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
class M545KinematicModelWithJoints : public KinematicModelWithJoints
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

  //todo it is shadowning the enum from towr
  enum LimbStartIndex
  {
    LF = J_LF_HAA,
    RF = J_RF_HAA,
    LH = J_LH_HFE,
    RH = J_RH_HFE,
    BOOM = J_TURN
  };

  static constexpr unsigned int numEE = 5;
  static constexpr unsigned int legDof = 3;
  static constexpr unsigned int boomDof = NUM_JOINTS - 4 * legDof;

  using MatrixXd = KinematicModelWithJoints::MatrixXd;
  using VectorXd = KinematicModelWithJoints::VectorXd;
  using JointLimitMap = std::unordered_map<std::string, std::unordered_map<std::string, double>>;
  using SparseMatrix = KinematicModelWithJoints::SparseMatrix;
  using EEJac = std::vector<SparseMatrix>;
  using LimbEnum = loco_m545::RD::LimbEnum;
  using EEorientation = EEPos;

  M545KinematicModelWithJoints() = delete;
  M545KinematicModelWithJoints(const std::string &urdfDescription, double dt);

  //todo remove this maybe
  bool EEhasWheel(int limbId);

  //update base stuff and joints
  void UpdateModel(VectorXd jointAngles, int limbId) final;

  /* base frame */
  // these are in the base frame
  Eigen::Vector3d GetEEPositionsBase(int limbId) final;

  SparseMatrix GetTranslationalJacobiansWRTjointsBase(int limbId) final;

  Eigen::Vector3d GetEEOrientationBase(int limbId) final;
  Eigen::Vector3d GetEEOrientationVectorBase(int limbId, int dim) final;
  SparseMatrix GetOrientationJacobiansWRTjointsBase(int limbId) final;
  SparseMatrix GetOrientationVectorJacobianBase(int limbId, int dim) final;


  VectorXd GetLowerJointLimits(int limbId) final;
  VectorXd GetUpperJointLimits(int limbId) final;

  int GetNumDof(int limbId) final;

  int GetNumDofTotal() final;

  EEPos GetNominalStanceInBase() const final;

  Vector3d GetMaximumDeviationFromNominal() const final;

  void printCurrentJointPositions();

  Vector3d GetBasePositionFromFeetPostions() final;

 private:

  void CalculateAngularVelocityJacobian(int limbId);

  Eigen::Matrix3d GetRotMat(int limbId);

  int getLimbStartingId(int LimbId);

  void CalculateJointLimits();
  void CalculateJointLimitsforSpecificLimb(const excavator_model::Limits &limtis,
                                           loco_m545::RD::LimbEnum limb, unsigned int dof);
  //update joints
  void UpdateModel(const VectorXd &jointAngles);

  void PrintJointLimits();


  void UpdateSpecificLimb(loco_m545::RD::LimbEnum limb, const VectorXd &jointAngles,
                          unsigned int dof);

  void ExtractJointJacobianEntries(const MatrixXd &bigJacobian, loco_m545::RD::LimbEnum limb,
                                   LimbStartIndex limbStartIndex, unsigned int dof,
                                   EEJac &jacArray);

  void CalculateTranslationalJacobiansWRTjointsBase(int limbId);


  void CalculateRotationalJacobiansWRTjointsBase(int limbId);

  excavator_model::ExcavatorModel model_;
  JointLimitMap joint_limits_;
  VectorXd upper_joint_limits_;
  VectorXd lower_joint_limits_;

  Eigen::Vector3d rotMat2ypr(const Eigen::Matrix3d &mat);

  SparseMatrix angularVelocity2eulerDerivativesMat(const Vector3d &ypr);





  std::vector<int> num_dof_limbs_ { legDof, legDof, legDof, legDof, boomDof };
  Vector3d euler_ypr_;
  Vector3d base_xyz_;
  VectorXd joints_;

  //base frame
  EEPos ee_pos_base_;
  EEJac ee_trans_jac_joints_base_;
  EEorientation ee_ypr_;
  EEJac ee_orientation_jac_base_;

};

int M545KinematicModelWithJoints::GetNumDof(int limbId)
{
  return num_dof_limbs_.at(limbId);
}

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

