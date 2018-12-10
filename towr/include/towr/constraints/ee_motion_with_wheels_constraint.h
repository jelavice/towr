/*
 * ee_motion_with_wheels_constraint.h
 *
 *  Created on: Dec 6, 2018
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

class EEMotionWithWheelsConstraint : public TimeDiscretizationConstraint
{
 public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;
  using SparseVector = Eigen::SparseVector<double,Eigen::RowMajor>;

  EEMotionWithWheelsConstraint(KinematicModelWithJoints::Ptr robot_model, double T, double dt,
                                EE ee, const SplineHolder& spline_holder);

  ~EEMotionWithWheelsConstraint() = default;

 private:

  //todo remove this hardcoded constraint
  const double vel_component_max_ = 1.0; /* m/s */

  EE ee_;

  NodeSpline::Ptr ee_with_wheels_motion_;
  NodeSpline::Ptr joints_motion_;
  EulerConverter base_angular_; ///< the orientation of the base.

  int num_constraints_per_node_;

  KinematicModelWithJoints::Ptr kinematic_model_;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const final;
  void UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const final;
  void UpdateJacobianAtInstance(double t, int k, std::string var_set, Jacobian&) const final;

  int GetRow(int node) const;



};

}
/* namesapce*/

