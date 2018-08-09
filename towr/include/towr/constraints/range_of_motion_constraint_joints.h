/*
 * range_of_motion_constraint_complicated.h
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/euler_converter.h>

#include <towr/models/kinematic_model.h>
#include "time_discretization_constraint.h"

namespace towr {

class RangeOfMotionConstraintJoints : public TimeDiscretizationConstraint
{
 public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;

  RangeOfMotionConstraintJoints(const KinematicModel::Ptr& robot_model, double T, double dt,
                                const EE& ee, const SplineHolder& spline_holder);

  ~RangeOfMotionConstraintJoints() = default;

 private:

  NodeSpline::Ptr base_linear_;     ///< the linear position of the base.
  EulerConverter base_angular_; ///< the orientation of the base.
  NodeSpline::Ptr ee_motion_;       ///< the linear position of the endeffectors.

  Eigen::Vector3d max_deviation_from_nominal_;
  Eigen::Vector3d nominal_ee_pos_B_;
  EE ee_;

  NodeSpline::Ptr joints_motion_;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance(double t, int k, VecBound&) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

  int GetRow(int node, int dimension) const;

};

}
/* namesapce*/