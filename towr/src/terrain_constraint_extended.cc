/*
 * terrain_constraint_extended.cc
 *
 *  Created on: Dec 7, 2018
 *      Author: jelavice
 */

#include <towr/constraints/terrain_constraint_extended.h>

namespace towr {

TerrainConstraintExtended::TerrainConstraintExtended(const HeightMap::Ptr& terrain,
                                                     std::string ee_motion, int ee_id,
                                                     bool ee_has_wheel)
    : ConstraintSet(kSpecifyLater, "terrain-" + ee_motion)
{
  ee_motion_id_ = ee_motion;
  terrain_ = terrain;
  ee_ = ee_id;
  ee_has_wheel_ = ee_has_wheel;

}

void TerrainConstraintExtended::InitVariableDependedQuantities(const VariablesPtr& x)
{
  //todo change this such that we get components of the motion and wheel

  if (ee_has_wheel_ == false)
    ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_id_);
  else
    ee_with_wheel_motion_ = x->GetComponent<NodesVariablesEEMotionWithWheels>(ee_motion_id_);

  std::vector<Node> nodes;
  GetNodes(&nodes);

  //todo see what happens when I have my time discretized nodes
  // do I need to skip first node or not??
  // skip first node, b/c already constrained by initial stance
  for (int id = 1; id < nodes.size(); ++id)
    node_ids_.push_back(id);

  int constraint_count = node_ids_.size();
  SetRows(constraint_count);
}

Eigen::VectorXd TerrainConstraintExtended::GetValues() const
{
  VectorXd g(GetRows());

  std::vector<Node> nodes;
  GetNodes(&nodes);

  int row = 0;
  for (int id : node_ids_) {
    Vector3d p = nodes.at(id).p();
    g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  return g;
}

TerrainConstraintExtended::VecBound TerrainConstraintExtended::GetBounds() const
{
  VecBound bounds(GetRows());
  double max_distance_above_terrain = 1e20;  // [m]

  int row = 0;
  for (int id : node_ids_) {

    //todo make this ifs nicer if you can
    if (ee_has_wheel_) {
      if (ee_with_wheel_motion_->GetNonPhaseNodeInfoAt(id).isInContact_)
        bounds.at(row) = ifopt::BoundZero;
      else
        bounds.at(row) = ifopt::Bounds(0.0, max_distance_above_terrain);
    } else {
      if (ee_motion_->IsConstantNode(id))
        bounds.at(row) = ifopt::BoundZero;
      else
        bounds.at(row) = ifopt::Bounds(0.0, max_distance_above_terrain);
    }
    row++;
  }

  return bounds;
}

void TerrainConstraintExtended::FillJacobianBlock(std::string var_set, Jacobian& jac) const
{
  if (ee_has_wheel_) {

    if (var_set == ee_with_wheel_motion_->GetName()) {
      auto nodes = ee_with_wheel_motion_->GetNodes();
      int row = 0;
      for (int id : node_ids_) {
        int idx = ee_with_wheel_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Z));
        jac.coeffRef(row, idx) = 1.0;

        Vector3d p = nodes.at(id).p();
        for (auto dim : { X, Y }) {
          int idx = ee_with_wheel_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, dim));
          jac.coeffRef(row, idx) = -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());
        }
        row++;
      }
    }

  } else { //todo factor out the common code
    if (var_set == ee_motion_->GetName()) {
      auto nodes = ee_motion_->GetNodes();
      int row = 0;
      for (int id : node_ids_) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Z));
        jac.coeffRef(row, idx) = 1.0;

        Vector3d p = nodes.at(id).p();
        for (auto dim : { X, Y }) {
          int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, dim));
          jac.coeffRef(row, idx) = -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());
        }
        row++;
      }
    }
  }
}

void TerrainConstraintExtended::GetNodes(std::vector<Node> *nodes) const
{

  if (ee_has_wheel_)
    *nodes = ee_with_wheel_motion_->GetNodes();
  else
    *nodes = ee_motion_->GetNodes();

}

} /* namespace towr */
