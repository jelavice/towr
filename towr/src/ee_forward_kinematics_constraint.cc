/*
 * forward_kinematics_constraint.cc
 *
 *  Created on: Dec 5, 2018
 *      Author: jelavice
 */

#include <towr/variables/variable_names.h>
#include "towr/constraints/ee_forward_kinematics_constraint.h"
#include "towr/variables/spline_holder_extended.h"

namespace towr {

//todo get rid of the wheel directinal constraints
EEforwardKinematicsConstraint::EEforwardKinematicsConstraint(KinematicModelWithJoints::Ptr model,
                                                             double T, double dt, EE ee,
                                                             const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "forwardkinematics-" + std::to_string(ee))
{
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);

  ee_ = ee;

  const SplineHolderExtended *spline_holder_ptr = spline_holder.as<SplineHolderExtended>();

  if (spline_holder_ptr == nullptr)
    throw std::runtime_error("Couldn't convert from spline_holder to spline_holder_extended");

  joints_motion_ = spline_holder_ptr->joint_motion_.at(ee_);

  kinematic_model_ = model;

  if (model->EEhasWheel(ee))
    ee_with_wheels_motion_ = spline_holder.as<SplineHolderExtended>()->ee_with_wheels_motion_.at(
        mapToSplineHolderEEId(ee));
  else
    ee_motion_ = spline_holder.ee_motion_.at(mapToSplineHolderEEId(ee));

  //need to include the constraints for all the joint bounds as well
  num_constraints_per_node_ = dim3;  // position (3 position constraints)

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}

//this one is called first it seems
void EEforwardKinematicsConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{

  int rowStart = GetRow(k, 0);

  /* first get all the variables*/

  Vector3d ee_pos_W;
  ComputeEEpositionWorld(t, &ee_pos_W);

  Vector3d base_W = base_linear_->GetPoint(t).p();

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  Vector3d vector_base_to_ee_W = ee_pos_W - base_W;
  Vector3d vector_base_to_ee_B = b_R_w * (vector_base_to_ee_W);

  VectorXd joint_positions = joints_motion_->GetPoint(t).p();
  kinematic_model_->UpdateModel(joint_positions, ee_);

  Vector3d pos_ee_joints_B = kinematic_model_->GetEEPositionBase(ee_);

  //endeffector position
  g.middleRows(rowStart, dim3) = pos_ee_joints_B - vector_base_to_ee_B;

}
void EEforwardKinematicsConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{

  int rowStart = GetRow(k, 0);

//now workout the endeffector constraint bounds
//equality constarints
  for (int dim = 0; dim < dim3; ++dim) {
    bounds.at(rowStart++) = ifopt::BoundZero;
  }

}

void EEforwardKinematicsConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                             Jacobian& jac) const
{

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  int row_start = GetRow(k, 0);

  if (var_set == id::EEJointNodes(ee_)) {

    VectorXd joint_positions = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_positions, ee_);

    jac.middleRows(row_start, dim3) = kinematic_model_->GetTranslationalJacobiansWRTjointsBase(ee_)
        * joints_motion_->GetJacobianWrtNodes(t, kPos);

  }

  if (var_set == id::base_lin_nodes) {

    //work out the end-effector constraint
    jac.middleRows(row_start, dim3) = b_R_w * base_linear_->GetJacobianWrtNodes(t, kPos);

  }

  if (var_set == id::base_ang_nodes) {

    //work out the end-effector constraint
    Vector3d base_W = base_linear_->GetPoint(t).p();
    Vector3d ee_pos_W;
    ComputeEEpositionWorld(t, &ee_pos_W);
    Vector3d r_W = -1 * (ee_pos_W - base_W);
    jac.middleRows(row_start, dim3) = base_angular_.DerivOfRotVecMult(t, r_W, true);

  }

  if (var_set == id::EEMotionWithWheelsNodes(ee_)) {

    jac.middleRows(row_start, dim3) = -1 * b_R_w
        * ee_with_wheels_motion_->GetJacobianWrtNodes(t, kPos);

  }

  if (var_set == id::EEMotionNodes(ee_)) {

    jac.middleRows(row_start, dim3) = -1 * b_R_w * ee_motion_->GetJacobianWrtNodes(t, kPos);

  }

}

int EEforwardKinematicsConstraint::GetRow(int node, int dimension) const
{
  return node * num_constraints_per_node_ + dimension;

}

void EEforwardKinematicsConstraint::ComputeEEpositionWorld(double t, Vector3d *pos_ee_W) const
{

  if (kinematic_model_->EEhasWheel(ee_))
    *pos_ee_W = ee_with_wheels_motion_->GetPoint(t).p();
  else
    *pos_ee_W = ee_motion_->GetPoint(t).p();
}

int EEforwardKinematicsConstraint::mapToSplineHolderEEId(int absolute_ee_id) const
{

  if (kinematic_model_ == nullptr)
    throw std::runtime_error("Modle ptr not initialized");

  //todo establish a convention, document it and remove the magic number
  if (kinematic_model_->EEhasWheel(absolute_ee_id))
    return absolute_ee_id;
  else
    return absolute_ee_id - 4;
  // 4 = num of feets with wheels, they are enumerated before the boom

}

}
/*namespace*/

