/*
 * joint_limits_constraint.h
 *
 *  Created on: Dec 5, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/euler_converter.h>

#include <towr/models/kinematic_model_with_joints.h>
#include <towr/models/kinematic_model.h>
#include "time_discretization_constraint.h"

namespace towr {

class EEjointLimitsConstraint : public TimeDiscretizationConstraint
{
 public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;
  using SparseVector = Eigen::SparseVector<double,Eigen::RowMajor>;

  EEjointLimitsConstraint(KinematicModelWithJoints::Ptr robot_model, double T, double dt,
                                EE ee, const SplineHolder& spline_holder);

  ~EEjointLimitsConstraint() = default;

 private:

  //todo remove this
  const double max_joint_vel_ = 0.5; /* rad/s */

  int num_dof_;
  EE ee_;

  NodeSpline::Ptr joints_motion_;

  int num_constraints_per_node_;

  KinematicModelWithJoints::Ptr kinematic_model_;

  Eigen::VectorXd lower_bounds_;
  Eigen::VectorXd upper_bounds_;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const final;
  void UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const final;
  void UpdateJacobianAtInstance(double t, int k, std::string var_set, Jacobian&) const final;

  int GetRow(int node) const;



};

}
/* namesapce*/
