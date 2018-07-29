/*
 * wheel_direction_constraint.cc
 *
 *  Created on: Jul 26, 2018
 *      Author: jelavice
 */

#include <towr/constraints/wheel_direction_constraint.h>

#include <towr/variables/variable_names.h>

#include <iostream>

namespace towr {

WheelDirectionConstraint::WheelDirectionConstraint(EE ee, const SplineHolder *spline_holder)
    : ifopt::ConstraintSet(kSpecifyLater, "wheel-" + id::WheelAngleNodes(ee))
{

  n_constraints_per_node_ = 4;  // wheel direction on the x and y
  ee_ = ee;
  spline_holder_ = spline_holder;

}

void WheelDirectionConstraint::InitVariableDependedQuantities(const VariablesPtr& x)
{
  ee_wheel_angles_ = x->GetComponent<NodesVariablesPhaseBased>(id::WheelAngleNodes(ee_));
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(id::EEMotionNodes(ee_));  //this are swing or driving nodes

  stance_nodes_ids_ = ee_wheel_angles_->GetIndicesOfNonConstantNodes();

  int constraint_count = stance_nodes_ids_.size() * n_constraints_per_node_;
  SetRows(constraint_count);
}

//todo make it work when number of nodes are not the same
Eigen::VectorXd WheelDirectionConstraint::GetValues() const
{
  VectorXd g(GetRows());

  int row = 0;
  auto angle_nodes = ee_wheel_angles_->GetNodes();
  auto motion_nodes = ee_motion_->GetNodes();
  for (int node_id : stance_nodes_ids_) {

    Vector3d v = motion_nodes.at(node_id).v();

    Eigen::VectorXd angleTemp = angle_nodes.at(node_id).p();  // eigen type

    //std::cout << "Agnle size: " << angleTemp.size() << std::endl;
    double angle = angleTemp(0);

    //first get the phase durations
    auto phase_durations = spline_holder_->phase_durations_.at(ee_)->GetPhaseDurations();

    //first get the time
    double time_at_node = ee_wheel_angles_->GetTimeAtCurrentNode(node_id, phase_durations);

    //get current base yaw angle
    double base_yaw = spline_holder_->base_angular_->GetPoint(time_at_node).p()(Z);

    //std::cout << "Wheel angle: " << angle << std::endl;

    //now add more constraints

    g(row++) = angle - base_yaw;
    g(row++) = v.x() * std::sin(angle) - v.y() * std::cos(angle);
    g(row++) = v.x();
    g(row++) = v.y();


  }

  return g;
}

WheelDirectionConstraint::VecBound WheelDirectionConstraint::GetBounds() const
{
  VecBound bounds;

  for (int f_node_id : stance_nodes_ids_) {
    bounds.push_back(ifopt::Bounds(-max_turning_angle_, max_turning_angle_));
    bounds.push_back(ifopt::BoundZero);
    bounds.push_back(ifopt::Bounds(-max_velocity, max_velocity));
    bounds.push_back(ifopt::Bounds(-max_velocity, max_velocity));
  }

  return bounds;
}

//TODO accomodate for new constraints
void WheelDirectionConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac) const
{

  if (var_set == ee_wheel_angles_->GetName()) {

    int row = 0;
    auto angle_nodes = ee_wheel_angles_->GetNodes();
    auto motion_nodes = ee_motion_->GetNodes();
    for (int node_id : stance_nodes_ids_) {

      int row_reset = row;
      Vector3d v = motion_nodes.at(node_id).v();
      double angle = angle_nodes.at(node_id).p()(0);  // eigen Xd type

      auto dim = X;
      int idx = ee_wheel_angles_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kPos, dim));

      jac.coeffRef(row_reset++, idx) = 1.0;
      jac.coeffRef(row_reset++, idx) = v.x() * std::cos(angle) + v.y() * std::sin(angle);  // unilateral force

      row += n_constraints_per_node_;
    }
  }

  if (var_set == ee_motion_->GetName()) {

    int row = 1;
    auto angle_nodes = ee_wheel_angles_->GetNodes();
    auto motion_nodes = ee_motion_->GetNodes();
    for (int node_id : stance_nodes_ids_) {

      double angle = angle_nodes.at(node_id).p()(0);  // eigen Xd type

      Vector3d derivative(std::sin(angle), -std::cos(angle), 0.0);


      //the complicated velocity constraint
      int row_reset = row;
      for (auto dim : { X, Y }) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kVel, dim));
        jac.coeffRef(row_reset, idx) = derivative(dim);
      }
      ++row_reset;

      for (auto dim : { X, Y }) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kVel, dim));
        jac.coeffRef(row_reset++, idx) = 1.0;
      }

      row += n_constraints_per_node_;
    }
  }

  if (var_set == id::base_ang_nodes) {

    int row = 0;

    for (int node_id : stance_nodes_ids_) {

      auto phase_durations = spline_holder_->phase_durations_.at(ee_)->GetPhaseDurations();

      double time_at_node = ee_wheel_angles_->GetTimeAtCurrentNode(node_id, phase_durations);

      auto jacobian = spline_holder_->base_angular_->GetJacobianWrtNodes(time_at_node, kPos);

      //update angular rows in the big jacobian
      //read only the last row of the small jacobian
      jac.middleRows(row, 1) = -jacobian.middleRows(Z, 1);

      row += n_constraints_per_node_;
    }

  }

}

} /* namespace towr */

