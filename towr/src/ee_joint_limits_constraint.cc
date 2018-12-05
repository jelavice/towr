/*
 * joint_limits_constraint.cc
 *
 *  Created on: Dec 5, 2018
 *      Author: jelavice
 */

#include <towr/variables/variable_names.h>
#include "towr/constraints/ee_joint_limits_constraint.h"
#include "towr/variables/spline_holder_extended.h"

namespace towr {

EEjointLimitsConstraint::EEjointLimitsConstraint(KinematicModelWithJoints::Ptr model,
                                                           double T, double dt, EE ee,
                                                           const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "jointlimits-" + std::to_string(ee))
{

  ee_ = ee;

  const SplineHolderExtended *spline_holder_ptr = spline_holder.as<SplineHolderExtended>();

  if (spline_holder_ptr == nullptr)
    throw std::runtime_error("Couldn't convert from spline_holder to spline_holder_extended");

  joints_motion_ = spline_holder_ptr->joint_motion_.at(ee_);

  kinematic_model_ = model;
  num_dof_ = kinematic_model_->GetNumDof(ee_);

  num_constraints_per_node_ = 2 * kinematic_model_->GetNumDof(ee_);  //for the range and for the velocity

  //cache these guys
  lower_bounds_ = kinematic_model_->GetLowerJointLimits(ee_);
  upper_bounds_ = kinematic_model_->GetUpperJointLimits(ee_);

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}

void EEjointLimitsConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  int row_start = GetRow(k);

  g.middleRows(row_start, num_dof_) = joints_motion_->GetPoint(t).p();
  row_start += num_dof_;
  g.middleRows(row_start, num_dof_) = joints_motion_->GetPoint(t).v();

}
void EEjointLimitsConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{

  int row_start = GetRow(k);
  //todo remove the hack for the boom,
  if (ee_ == 4) { // if this is boom then do

    Eigen::VectorXd boom_bounds;

    // clamp the joint angles for the boom
    // order [TURN, BOOM, DIPPER, TELE, EE_PITCH]
    if (num_dof_ == 4) {
      boom_bounds.resize(num_dof_);
      boom_bounds << 0.0, -1.2, 2.0, 0.0;
    } else {
      boom_bounds.resize(num_dof_);
      boom_bounds << 0.0, -1.2, 2.0, 0.0, 2.2;
    }

    ifopt::Bounds b;
    for (int i = 0; i < boom_bounds.size(); ++i) {
      bounds.at(row_start + i) = ifopt::Bounds(boom_bounds(i), boom_bounds(i));  //update joint position bounds
    }

  } else { //if these are legs then do

    for (int i = 0; i < num_dof_; ++i) {
      bounds.at(row_start + i) = ifopt::Bounds(lower_bounds_(i), upper_bounds_(i));  //update joint position bounds
    }

  }

  row_start += num_dof_;

  for (int i = 0; i < num_dof_; ++i) {
    bounds.at(row_start + i) = ifopt::Bounds(-max_joint_vel_, max_joint_vel_);  // update joint velocity bounds
  }

}
void EEjointLimitsConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                            Jacobian& jac) const
{

  if (var_set == id::EEJointNodes(ee_)) {
    int row_start = GetRow(k);

    jac.middleRows(row_start, num_dof_) = joints_motion_->GetJacobianWrtNodes(t, kPos);
    row_start += num_dof_;
    jac.middleRows(row_start, num_dof_) = joints_motion_->GetJacobianWrtNodes(t, kVel);

  }

}

int EEjointLimitsConstraint::GetRow(int node) const
{
  return node * num_constraints_per_node_;

}

} /* namespace */


