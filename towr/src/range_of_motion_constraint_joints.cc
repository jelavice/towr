/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/range_of_motion_constraint_joints.h>
#include <towr/variables/variable_names.h>

namespace towr {

RangeOfMotionConstraintJoints::RangeOfMotionConstraintJoints(KinematicModelJoints::Ptr model,
                                                             double T, double dt, EE ee,
                                                             const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "rangeofmotionjoints-" + std::to_string(ee))
{
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);

  ee_motion_ = spline_holder.ee_motion_.at(ee);
  joints_motion_ = spline_holder.joint_motion_.at(ee_);

  kinematic_model_ = std::move(model);

  ee_ = ee;

  //todo make this work for all terrain (also inclinations), gonna need more constraints
  num_constraints_per_node_ = k3D + 1;  // position (3 position constraints, and a yaw angle of the wheel)

  //need to include the constraints for all the joint bounds as well
  num_constraints_per_node_ += kinematic_model_->GetNumDof(ee_);

  //cache these guys
  lower_bounds_ = kinematic_model_->GetLowerJointLimits(ee_);
  upper_bounds_ = kinematic_model_->GetUpperJointLimits(ee_);

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}


//this one is called first it seems
void RangeOfMotionConstraintJoints::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{

  int dimension = 0;

  /* first get all the variables*/
  //todo make sure that the caller has set the right size of the state
  VectorXd joint_positions = joints_motion_->GetPoint(t).p();

  // now update the value of the ee constraint
  // first get the position and euler shit
  Vector3d base_position = base_linear_->GetPoint(t).p();
  Vector3d base_angular = base_angular_.GetEulerAngles(t);

  // second update the model
  kinematic_model_->UpdateModel(joint_positions, base_angular, base_position);

  //then get all the positions and shit that I need
  Vector3d ee_position_from_joints = kinematic_model_->GetEEPositionsWorld().at(ee_);
  Vector3d ee_position = ee_motion_->GetPoint(t).p();

  //take out the yaw
  double wheel_yaw = kinematic_model_->GetEEOrientation().at(ee_).z();

  //put it in the constraint
  Eigen::Vector2d ee_vel = ee_motion_->GetPoint(t).v().segment(0, 2);  // get the x and y velocity

  /*now update the actual constraints*/
  //joint range constraint
  g.middleRows(GetRow(k, dimension), joint_positions.size()) = joint_positions;
  dimension += joint_positions.size();

  //endeffector position
  g.middleRows(GetRow(k, dimension), dim3) = ee_position_from_joints - ee_position;
  dimension += dim3;

  //no slip in lateral direction
  g(GetRow(k, dimension)) = ee_vel.x() * std::sin(wheel_yaw) - ee_vel.y() * std::cos(wheel_yaw);  // last row anyway

}
void RangeOfMotionConstraintJoints::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  int dimension = 0;

  //first work out the joint bounds
  for (int j = 0; j < kinematic_model_->GetNumDof(ee_); ++j) {
    ifopt::Bounds b;
    b.lower_ = lower_bounds_(j);
    b.upper_ = upper_bounds_(j);
    bounds.at(GetRow(k, dimension++)) = b;
  }

  //now workout the endeffector constraint bounds
  //equality constarints
  for (int dim = 0; dim < dim3; ++dim) {
    bounds.at(GetRow(k, dimension++)) = ifopt::BoundZero;
  }

  //then workout the heading direction
  //equality constraint
  bounds.at(GetRow(k, dimension)) = ifopt::BoundZero;

}


void RangeOfMotionConstraintJoints::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                             Jacobian& jac) const
{

  // need

  // 1. joints here I should also fit the end yaw angle constraint

  // 2. base lin

  // 3. base ang

  // 4. end effector position and end effectors felocity

  //start with the joints since it is the easiest

  int row_start = GetRow(k, 0);

  if (var_set == id::JointNodes(ee_)) {

    // jacobian wrt to joints
    int dim = kinematic_model_->GetNumDof(ee_);
    jac.middleRows(row_start, dim) = joints_motion_->GetJacobianWrtNodes(t, kPos);
    row_start += dim;

    //now work out the end-effector constarint
    dim = dim3;
    jac.middleRows(row_start, dim) = kinematic_model_->GetTranslationalJacobiansWRTjoints().at(ee_)
        * joints_motion_->GetJacobianWrtNodes(t, kPos);
    row_start += dim;

    //need velocity and angle for the last constraint
    //dis is the wheel heading constraint
    dim = 1;
    Eigen::Vector2d ee_vel = ee_motion_->GetPoint(t).v().segment(0, 2);  // get the x and y velocity
    double yaw = kinematic_model_->GetEEOrientation().at(ee_).z();

    jac.middleRows(row_start, dim) = kinematic_model_->GetOrientationJacobiansWRTjoints().at(ee_)
        .row(2) * (ee_vel.x() * std::cos(yaw) + ee_vel.y() * std::sin(yaw));
  }

  if (var_set == id::base_lin_nodes) {
    //skip the first constraint since the derivative is zero wrt to base lin nodes

    //work out the end-effector constraint
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint
    int dim = dim3;
    jac.middleRows(row_start, dim) = kinematic_model_->GetTranslationalJacobianWRTbasePosition().at(
        ee_) * base_linear_->GetJacobianWrtNodes(t, kPos);
    row_start += dim;

    // jacobian of the last constraint is also zero wrt to the base lin nodes

  }

  if (var_set == id::base_ang_nodes) {
    //skip the first constraint since the derivative is zero wrt to base ang nodes

    //work out the end-effector constraint
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint
    int dim = dim3;
    jac.middleRows(row_start, dim) = kinematic_model_->GetTranslatinalJacobianWRTbaseOrientation()
        .at(ee_) * base_angular_.GetNodeSpline()->GetJacobianWrtNodes(t, kPos);
    row_start += dim;

    // jacobian of the last constraint is also zero wrt to the base lin nodes
    dim = 1;
    Eigen::Vector2d ee_vel = ee_motion_->GetPoint(t).v().segment(0, 2);  // get the x and y velocity
    double yaw = kinematic_model_->GetEEOrientation().at(ee_).z();

    jac.middleRows(row_start, dim) = kinematic_model_->GetOrientationJacobiansWRTbaseOrientation()
        .at(ee_).row(2) * base_angular_.GetNodeSpline()->GetJacobianWrtNodes(t, kPos).row(2)
        * (ee_vel.x() * std::cos(yaw) + ee_vel.y() * std::sin(yaw));

  }

  if (var_set == id::EEMotionNodes(ee_)) {

    //skip the first constraint since the derivative is zero wrt to base ang nodes

    //work out the end-effector constraint
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint
    int dim = dim3;
    jac.middleRows(row_start, dim) = (-ee_motion_->GetJacobianWrtNodes(t, kPos)).eval();
    row_start += dim;

    //now work out the wheel direction constraint
    dim = 1;
    double yaw = kinematic_model_->GetEEOrientation().at(ee_).z();
    Eigen::Vector3d constraint_partial_derivative(std::cos(yaw), std::sin(yaw), 0.0);
    jac.middleRows(row_start, dim) = (constraint_partial_derivative * ee_motion_->GetJacobianWrtNodes(t, kPos)).eval().sparseView();

  }

}

int RangeOfMotionConstraintJoints::GetRow(int node, int dimension) const
{
  return node * num_constraints_per_node_ + dimension;

}

} /*namespace*/
