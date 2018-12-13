/*
 * forward_kinematics_constraint.h
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
#include "towr/constraints/time_discretization_constraint.h"

namespace towr {

class EEforwardKinematicsConstraint : public TimeDiscretizationConstraint
{
 public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;
  using SparseVector = Eigen::SparseVector<double,Eigen::RowMajor>;

  EEforwardKinematicsConstraint(KinematicModelWithJoints::Ptr robot_model, double T, double dt,
                                EE ee, const SplineHolder& spline_holder);

  ~EEforwardKinematicsConstraint() = default;

 private:

  NodeSpline::Ptr base_linear_;     ///< the linear position of the base.
  EulerConverter base_angular_; ///< the orientation of the base.
  NodeSpline::Ptr ee_with_wheels_motion_, ee_motion_;       ///< the linear position of the endeffectors.

  //todo add the boom here

  EE ee_;
  bool ee_has_wheel_;

  NodeSpline::Ptr joints_motion_;

  int num_constraints_per_node_;

  KinematicModelWithJoints::Ptr kinematic_model_;

  const int dim3 = 3;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const final;
  void UpdateBoundsAtInstance(double t, int k, VecBound&) const final;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const final;

  int GetRow(int node) const;

  void ComputeEEpositionWorld(double t, Vector3d *pos_ee_W) const;

  int EEId2SplineHolderEEId(int absolute_ee_id) const;



};

}
/* namesapce*/
