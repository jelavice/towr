/*
 * terrain_constraint_extended.h
 *
 *  Created on: Dec 7, 2018
 *      Author: jelavice
 */

#pragma once

#include <ifopt/constraint_set.h>

#include <towr/variables/nodes_variables_phase_based.h>
#include <towr/terrain/height_map.h>
#include <towr/variables/nodes_variables_all.h>

namespace towr {


class TerrainConstraintExtended : public ifopt::ConstraintSet {
public:
  using Vector3d = Eigen::Vector3d;

  /**
   * @brief Constructs a terrain constraint.
   * @param terrain  The terrain height value and slope for each position x,y.
   * @param ee_motion_id The name of the endeffector variable set.
   */
  TerrainConstraintExtended (const HeightMap::Ptr& terrain, std::string ee_motion_id, int ee_id, bool ee_has_wheel );
  virtual ~TerrainConstraintExtended () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:

  void GetNodes(std::vector<Node> *nodes) const;

  NodesVariablesPhaseBased::Ptr ee_motion_; ///< the position of the endeffector.
  NodesVariablesAll::Ptr ee_with_wheel_motion_; // position of the endeffector with wheel
  HeightMap::Ptr terrain_;    ///< the height map of the current terrain.
  bool ee_has_wheel_;


  unsigned int ee_;
  std::string ee_motion_id_;  ///< the name of the endeffector variable set.
  std::vector<int> node_ids_; ///< the indices of the nodes constrained.
};

} /* namespace towr */

