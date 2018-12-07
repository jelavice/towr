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

  //todo implement this guy

  //get joints from the spline

  Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();

  //update model

  kinematic_model_->UpdateModel(joint_angles, ee_);

  //get orientation of the wheels axis in the world frame

  Eigen::Vector3d wheel_axis_base = kinematic_model_->GetWheelAxisBase(ee_);

  //get rotation matrix world to base from base_angular spline

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  //get the ee motion with wheels velocity nodes

  Eigen::Vector3d ee_velocity_world = ee_with_wheels_motion_->GetPoint(t).v();

  // rotate them to the base frame

  // take the dot product

  g.middleRows(row_start, 1) = b_R_w * ee_velocity_world.sparseView()
      * wheel_axis_base.sparseView();

}
void EEMotionWithWheelsConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{

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

  int row_start = GetRow(k);
  //variable set for the base angles
  //update jacobian

  if (var_set == id::base_ang_nodes) {
    Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_angles, ee_);
    EulerConverter::JacobianRow wheel_axis_base = kinematic_model_->GetWheelAxisBase(ee_).sparseView();
    Eigen::Vector3d ee_motion_vel = ee_with_wheels_motion_->GetPoint(t).v();
    jac.middleRows(row_start, 1) = wheel_axis_base.transpose()
        * base_angular_.DerivOfRotVecMult(t, ee_motion_vel, true);
  }

  //variable set joints for this specific ee
  //update jacobian
  if (var_set == id::EEJointNodes(ee_)) {
    Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_angles, ee_);
    EulerConverter::JacobianRow ee_motion_vel = ee_with_wheels_motion_->GetPoint(t).v().sparseView();
    EulerConverter::MatrixSXd w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);
    jac.middleRows(row_start, 1) = ee_motion_vel.transpose() * w_R_b
        * kinematic_model_->GetWheelAxisJacobianBase(ee_)
        * joints_motion_->GetJacobianWrtNodes(t, kPos);
  }

  //variable set for ee motin with wheels
  //do the same again

  if (var_set == id::EEMotionWithWheelsNodes(ee_)) {
    jac.middleRows(row_start, k3D) = Eigen::Vector3d().setOnes().sparseView();
    row_start += k3D;
    Eigen::VectorXd joint_angles = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_angles, ee_);
    EulerConverter::JacobianRow wheel_axis_base = kinematic_model_->GetWheelAxisBase(ee_).sparseView();
    EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();
    jac.middleRows(row_start, 1) = wheel_axis_base.transpose() * b_R_w;
  }

}

int EEMotionWithWheelsConstraint::GetRow(int node) const
{
  return node * num_constraints_per_node_;

}

} /* namespace */

