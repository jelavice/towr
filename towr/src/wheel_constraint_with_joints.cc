/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/wheel_constraint_with_joints.h>
#include <towr/variables/variable_names.h>

namespace towr {


//todo get rid of the wheel directinal constraints
WheelConstraintWithJoints::WheelConstraintWithJoints(KinematicModelJoints::Ptr model,
                                                             double T, double dt, EE ee,
                                                             const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "wheelconstraintjoints-" + std::to_string(ee))
{
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);

  ee_ = ee;

  ee_motion_ = spline_holder.ee_motion_.at(ee_);
  joints_motion_ = spline_holder.joint_motion_.at(ee_);

  kinematic_model_ = model;

  //todo make this work for all terrain (also inclinations), gonna need more constraints

  //need to include the constraints for all the joint bounds as well
  num_constraints_per_node_ = 1;

  num_constraints_per_node_  += dim3; // limit all the velocities for ee speed

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}

//this one is called first it seems
void WheelConstraintWithJoints::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{

  int rowStart = GetRow(k, 0);

  /* first get all the variables*/

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  VectorXd joint_positions = joints_motion_->GetPoint(t).p();
  kinematic_model_->UpdateModel(joint_positions, ee_);

  Eigen::Vector3d ee_vel = ee_motion_->GetPoint(t).v();  // get the x and y  and the z velocity
  Eigen::Vector3d lateral_direction = GetLateralWheelHeading();

  //no slip in lateral direction
  g(rowStart++) = lateral_direction.transpose() * b_R_w * ee_vel;  // last row anyway


  g(rowStart++) = ee_vel.x();
  g(rowStart++) = ee_vel.y();
  g(rowStart++) = ee_vel.z();

}
void WheelConstraintWithJoints::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{

  int rowStart = GetRow(k, 0);


  //heading direction constraint
  bounds.at(rowStart++) = ifopt::BoundZero;

  for (int i =0; i < dim3; ++i){
    bounds.at(rowStart++) = ifopt::Bounds(-max_velocity_, max_velocity_);
  }




}

void WheelConstraintWithJoints::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                             Jacobian& jac) const
{

  EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  int row_start = GetRow(k, 0);

  if (var_set == id::JointNodes(ee_)) {

    VectorXd joint_positions = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_positions, ee_);

    Vector3d ee_vel = ee_motion_->GetPoint(t).v();

    double scalar = ((b_R_w * ee_vel).transpose() * GetLateralWheelHeadingDerivative())(0);

    Jacobian product = kinematic_model_->GetOrientationJacobiansWRTjointsBase(ee_).row(Z)
        * joints_motion_->GetJacobianWrtNodes(t, kPos);

    jac.row(row_start) = scalar * product;

  }



  if (var_set == id::base_ang_nodes) {

    Vector3d ee_vel = ee_motion_->GetPoint(t).v();

    // update the model
    VectorXd joint_positions = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_positions, ee_);

    auto lateral_wheel_heading = GetLateralWheelHeading().sparseView().transpose();

    jac.row(row_start) = lateral_wheel_heading.eval()
        * base_angular_.DerivOfRotVecMult(t, ee_vel, true);

  }

  if (var_set == id::EEMotionNodes(ee_)) {


    // update the model
    VectorXd joint_positions = joints_motion_->GetPoint(t).p();
    kinematic_model_->UpdateModel(joint_positions, ee_);

    Eigen::Vector3d ee_vel = ee_motion_->GetPoint(t).v();  // get the x and y velocity

    Jacobian lateral_wheel_heading = GetLateralWheelHeading().sparseView().transpose();
    Jacobian product = lateral_wheel_heading.eval() * b_R_w
        * ee_motion_->GetJacobianWrtNodes(t, kVel);

    jac.row(row_start++) = product;

    jac.middleRows(row_start, dim3) = ee_motion_->GetJacobianWrtNodes(t, kVel);

  }

}

int WheelConstraintWithJoints::GetRow(int node, int dimension) const
{
  return node * num_constraints_per_node_ + dimension;

}

Eigen::Vector3d WheelConstraintWithJoints::GetLateralWheelHeading() const
{
//take out the yaw
  double yaw = kinematic_model_->GetEEOrientationBase(ee_).z();

  return Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0.0);
}

Eigen::Vector3d WheelConstraintWithJoints::GetLateralWheelHeadingDerivative() const
{
//take out the yaw
  double yaw = kinematic_model_->GetEEOrientationBase(ee_).z();

  return Eigen::Vector3d(-std::cos(yaw), -std::sin(yaw), 0.0);
}

}
/*namespace*/
