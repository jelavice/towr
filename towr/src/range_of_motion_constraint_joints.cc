/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/range_of_motion_constraint_joints.h>
#include <towr/variables/variable_names.h>

namespace towr {


//todo get rid of the wheel directinal constraints
RangeOfMotionConstraintJoints::RangeOfMotionConstraintJoints(KinematicModelJoints::Ptr model,
                                                             double T, double dt, EE ee,
                                                             const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "rangeofmotionjoints-" + std::to_string(ee))
{
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);

  ee_ = ee;

  ee_motion_ = spline_holder.ee_motion_.at(ee_);
  joints_motion_ = spline_holder.joint_motion_.at(ee_);

  kinematic_model_ = model;

  //need to include the constraints for all the joint bounds as well
  num_constraints_per_node_ = kinematic_model_->GetNumDof(ee_) + dim3; // position (3 position constraints)

  //cache these guys
  lower_bounds_ = kinematic_model_->GetLowerJointLimits(ee_);
  upper_bounds_ = kinematic_model_->GetUpperJointLimits(ee_);

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}

//this one is called first it seems
void RangeOfMotionConstraintJoints::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{

  int rowStart = GetRow(k, 0);

  /* first get all the variables*/

  Vector3d pos_ee_W = ee_motion_->GetPoint(t).p();
  Vector3d base_W = base_linear_->GetPoint(t).p();

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  Vector3d vector_base_to_ee_W = pos_ee_W - base_W;
  Vector3d vector_base_to_ee_B = b_R_w * (vector_base_to_ee_W);

  VectorXd joint_positions = joints_motion_->GetPoint(t).p();
  kinematic_model_->UpdateModel(joint_positions, ee_);
  Vector3d pos_ee_joints_B = kinematic_model_->GetEEPositionsBase(ee_);

  /*now update the actual constraints*/

  //joint range constraint
  g.middleRows(rowStart, joint_positions.size()) = joint_positions;
  rowStart += joint_positions.size();

  //endeffector position
  g.middleRows(rowStart, dim3) = pos_ee_joints_B - vector_base_to_ee_B;

}
void RangeOfMotionConstraintJoints::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{

  int rowStart = GetRow(k, 0);

  //hack the joint limits for the boom
  //todo move this somewhere else
  if (ee_ == 4) {

    //no bound for the J_TURN
    ifopt::Bounds b;
    b = ifopt::NoBound;
    bounds.at(rowStart++) = b;

    //fix the J_BOOM
    b.lower_ = -1.2;
    b.upper_ = -1.2;
    bounds.at(rowStart++) = b;

    //fix the J_DIPPER
    b.lower_ = 2.0;
    b.upper_ = 2.0;
    bounds.at(rowStart++) = b;

    //fix the J_TELE
    b.lower_ = 0.0;
    b.upper_ = 0.0;
    bounds.at(rowStart++) = b;

    if (kinematic_model_->GetNumDof(ee_) > 4) {
      //fix the EE_PITCH
      b.lower_ = 2.2;
      b.upper_ = 2.2;
      bounds.at(rowStart++) = b;
    }

  } else {

    //first work out the joint bounds
    for (int j = 0; j < kinematic_model_->GetNumDof(ee_); ++j) {
      ifopt::Bounds b;
      b.lower_ = lower_bounds_(j);
      b.upper_ = upper_bounds_(j);
      bounds.at(rowStart++) = b;
    }
  }

  //now workout the endeffector constraint bounds
  //equality constarints
  for (int dim = 0; dim < dim3; ++dim) {
    bounds.at(rowStart++) = ifopt::BoundZero;
  }

}

void RangeOfMotionConstraintJoints::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                             Jacobian& jac) const
{

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  int row_start = GetRow(k, 0);

  if (var_set == id::JointNodes(ee_)) {

    VectorXd joint_positions = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_positions, ee_);

    Jacobian jacobianWrtNodes = joints_motion_->GetJacobianWrtNodes(t, kPos);

    // jacobian wrt to joints
    int dim = kinematic_model_->GetNumDof(ee_);
    jac.middleRows(row_start, dim) = jacobianWrtNodes;
    row_start += dim;

    //now work out the end-effector constarint

    jac.middleRows(row_start, dim3) = kinematic_model_->GetTranslationalJacobiansWRTjointsBase(ee_)
        * jacobianWrtNodes;

  }

  if (var_set == id::base_lin_nodes) {

    //skip the first constraint since the derivative is zero wrt to base lin nodes
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint

    //work out the end-effector constraint
    jac.middleRows(row_start, dim3) = b_R_w * base_linear_->GetJacobianWrtNodes(t, kPos);

  }

  if (var_set == id::base_ang_nodes) {

    //skip the first constraint since the derivative is zero wrt to base ang nodes
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint

    //work out the end-effector constraint
    Vector3d base_W = base_linear_->GetPoint(t).p();
    Vector3d ee_pos_W = ee_motion_->GetPoint(t).p();
    Vector3d r_W = -1 * (ee_pos_W - base_W);
    jac.middleRows(row_start, dim3) = base_angular_.DerivOfRotVecMult(t, r_W, true);

  }

  if (var_set == id::EEMotionNodes(ee_)) {

    //skip the first constraint since the derivative is zero wrt to base ang nodes
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint

    //work out the end-effector constraint
    jac.middleRows(row_start, dim3) = -1 * b_R_w * ee_motion_->GetJacobianWrtNodes(t, kPos);

  }

}

int RangeOfMotionConstraintJoints::GetRow(int node, int dimension) const
{
  return node * num_constraints_per_node_ + dimension;

}

}
/*namespace*/
