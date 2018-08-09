/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/range_of_motion_constraint_joints.h>

namespace towr {

RangeOfMotionConstraintJoints::RangeOfMotionConstraintJoints(KinematicModelJoints::Ptr model,
                                                             double T, double dt, const EE& ee,
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

void RangeOfMotionConstraintJoints::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{

  int dimension = 0;

  //todo make sure that the caller has set the right size of the state
  VectorXd joint_positions = joints_motion_->GetPoint(t).p();

  //copy values of de joints
  g.middleRows(GetRow(k,dimension), joint_positions.size()) = joint_positions;
  dimension += joint_positions.size();

  // now update the value of the ee constraint
  // first get the position and euler shit
  Vector3d base_position = base_linear_->GetPoint(t).p();
  //todo get rid of this conversions qut vs rotMat back and forth
  Vector3d base_anglular = KinematicModelJoints::rotMat2ypr(base_angular_.GetRotationMatrixBaseToWorld(t));

  // second update the model
  kinematic_model_->
  Vector3d constraint_value;
  //then get all athe positions and shit that I need




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
  //todo implement
}

int RangeOfMotionConstraintJoints::GetRow(int node, int dimension) const
{
  return node * num_constraints_per_node_ + dimension;

}

} /*namespace*/
