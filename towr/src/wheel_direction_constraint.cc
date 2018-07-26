/*
 * wheel_direction_constraint.cc
 *
 *  Created on: Jul 26, 2018
 *      Author: jelavice
 */

#include <towr/constraints/wheel_direction_constraint.h>

#include <towr/variables/variable_names.h>

namespace towr {

WheelDirectionConstraint::WheelDirectionConstraint(EE ee)
    : ifopt::ConstraintSet(kSpecifyLater, "wheel-" + id::WheelAngleNodes(ee))
{

  n_constraints_per_node_ = 1;  // wheel direction on the x and y
  ee_ = ee;
}

//todo add here the wheel forces

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
    double angle = angle_nodes.at(node_id).p()(0);  // eigen type

    g(row++) = v.x() * std::sin(angle) - v.y() * std::cos(angle);

  }

  return g;
}

WheelDirectionConstraint::VecBound WheelDirectionConstraint::GetBounds() const
{
  VecBound bounds;

  for (int f_node_id : stance_nodes_ids_) {
    bounds.push_back(ifopt::BoundZero);  // unilateral forces
  }

  return bounds;
}

//TODO implement this
void WheelDirectionConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac) const
{

  if (var_set == ee_wheel_angles_->GetName()) {
    int row = 0;
    auto angle_nodes = ee_wheel_angles_->GetNodes();
    auto motion_nodes = ee_motion_->GetNodes();
    for (int node_id : stance_nodes_ids_) {

      Vector3d v = motion_nodes.at(node_id).v();
      double angle = angle_nodes.at(node_id).p()(0);  // eigen Xd type

      auto dim = X;
      int idx = ee_wheel_angles_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kPos, dim));

      jac.coeffRef(row, idx) = v.x() * std::cos(angle) + v.y() * std::sin(angle);  // unilateral force

      row += n_constraints_per_node_;
    }
  }

  if (var_set == ee_motion_->GetName()) {
    int row = 0;
    auto angle_nodes = ee_wheel_angles_->GetNodes();
    auto motion_nodes = ee_motion_->GetNodes();
    for (int node_id : stance_nodes_ids_) {

      double angle = angle_nodes.at(node_id).p()(0);  // eigen Xd type

      Vector3d derivative(std::sin(angle), -std::cos(angle), 0.0);

      for (auto dim : { X, Y, Z }) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kVel, dim));
        jac.coeffRef(row, idx) = derivative(dim);  // unilateral force
      }

      row += n_constraints_per_node_;
    }
  }

}

} /* namespace towr */

