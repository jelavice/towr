/*
 * ee_motion_with_wheels_constraint.cc
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#include <towr/variables/variable_names.h>
#include "towr/constraints/ee_motion_with_wheels_constraint.h"
#include "towr/variables/spline_holder_extended.h"

namespace towr {

EEMotionWithWheelsConstraint::EEMotionWithWheelsConstraint(KinematicModelWithJoints::Ptr model,
                                                           double T, double dt, EE ee,
                                                           const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "eemotinowithwheels-" + std::to_string(ee))
{

  ee_ = ee;

  const SplineHolderExtended *spline_holder_ptr = spline_holder.as<SplineHolderExtended>();

  if (spline_holder_ptr == nullptr)
    throw std::runtime_error("Couldn't convert from spline_holder to spline_holder_extended");

  joints_motion_ = spline_holder_ptr->joint_motion_.at(ee_);
  ee_with_wheels_motion_ = spline_holder_ptr->ee_with_wheels_motion_.at(ee_);

  base_angular_ = EulerConverter(spline_holder.base_angular_);
  EulerConverter base_angular_;  ///< the orientation of the base.

  kinematic_model_ = model;

  num_constraints_per_node_ = k3D + 1;  // 3 constraints for all the velocity components, 1 constraint for the no lateral slip

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}

void EEMotionWithWheelsConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  int row_start = GetRow(k);

  g.middleRows(row_start, k3D) = ee_with_wheels_motion_->GetPoint(t).v();
  row_start += k3D;

  Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
  kinematic_model_->UpdateModel(joint_angles, ee_);

  Eigen::Vector3d ee_velocity_w = ee_with_wheels_motion_->GetPoint(t).v();

  Eigen::Matrix3d b_R_w(base_angular_.GetRotationMatrixBaseToWorld(t).transpose());
  Eigen::Matrix3d ee_R_b(kinematic_model_->GetRotationBaseToWheel(ee_));
  Eigen::Vector3d ee_velocity_ee = ee_R_b * b_R_w * ee_velocity_w;

  // take the dot product
  g(row_start) = ee_velocity_ee.y();  //lateral velocity
}
void EEMotionWithWheelsConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  //velocity bound constraint
  int row_start = GetRow(k);
  for (int dim = 0; dim < k3D; ++dim) {
    ifopt::Bounds b;
    b.upper_ = vel_component_max_;
    b.lower_ = -vel_component_max_;
    bounds.at(row_start++) = b;
  }

  bounds.at(row_start) = ifopt::BoundZero;

}
void EEMotionWithWheelsConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                            Jacobian& jac) const
{

  //variable set for the base angles
  //update jacobian

  if (var_set == id::base_ang_nodes) {
    int row_start = GetRow(k) + k3D;
    Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_angles, ee_);
    EulerConverter::MatrixSXd ee_R_b = kinematic_model_->GetRotationBaseToWheel(ee_).sparseView();
    Eigen::Vector3d ee_motion_vel = ee_with_wheels_motion_->GetPoint(t).v();
    jac.row(row_start) = ExtractRow(
        ee_R_b * base_angular_.DerivOfRotVecMult(t, ee_motion_vel, true), Y);
  }
//
//  //variable set joints for this specific ee
//  //update jacobian
  if (var_set == id::EEJointNodes(ee_)) {
    int row_start = GetRow(k) + k3D;
    Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_angles, ee_);
    Eigen::Vector3d ee_vel_w = ee_with_wheels_motion_->GetPoint(t).v();
    EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

    jac.row(row_start) = ExtractRow(kinematic_model_->GetDerivOfRotVecMult(b_R_w * ee_vel_w, ee_, false)
        * joints_motion_->GetJacobianWrtNodes(t, kPos),Y);
  }

  //variable set for ee motin with wheels
  //do the same again

  if (var_set == id::EEMotionWithWheelsNodes(ee_)) {
    int row_start = GetRow(k);
    jac.middleRows(row_start, k3D) = ee_with_wheels_motion_->GetJacobianWrtNodes(t, kVel);
    row_start += k3D;
    Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_angles, ee_);
    EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();
    EulerConverter::MatrixSXd ee_R_b = kinematic_model_->GetRotationBaseToWheel(ee_).sparseView();
    jac.row(row_start) = ExtractRow(
        ee_R_b * b_R_w * ee_with_wheels_motion_->GetJacobianWrtNodes(t, kVel), Y);
  }

}

int EEMotionWithWheelsConstraint::GetRow(int node) const
{
  return node * num_constraints_per_node_;

}

EEMotionWithWheelsConstraint::Jacobian EEMotionWithWheelsConstraint::ExtractRow(const Jacobian &jac,
                                                                                int row) const
{
  return jac.middleRows(row, 1);
}

} /* namespace */

