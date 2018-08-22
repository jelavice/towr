/*
 * WheelConstraintWithJoints.h
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/euler_converter.h>

#include <towr/models/kinematic_model_joints.h>
#include <towr/models/kinematic_model.h>
#include "time_discretization_constraint.h"

namespace towr {

class WheelConstraintWithJoints : public TimeDiscretizationConstraint
{
 public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;
  using SparseVector = Eigen::SparseVector<double,Eigen::RowMajor>;

  WheelConstraintWithJoints(KinematicModelJoints::Ptr robot_model, double T, double dt,
                                EE ee, const SplineHolder& spline_holder);

  ~WheelConstraintWithJoints() = default;

 private:

  NodeSpline::Ptr base_linear_;     ///< the linear position of the base.
  EulerConverter base_angular_; ///< the orientation of the base.
  NodeSpline::Ptr ee_motion_;       ///< the linear position of the endeffectors.

  EE ee_;

  NodeSpline::Ptr joints_motion_;

  int num_constraints_per_node_;

  KinematicModelJoints::Ptr kinematic_model_;

  const int dim3 = 3;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance(double t, int k, VecBound&) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

  int GetRow(int node, int dimension) const;

  Eigen::Vector3d GetLateralWheelHeading() const;
  Eigen::Vector3d GetLateralWheelHeadingDerivative() const;


};

}
/* namesapce*/
