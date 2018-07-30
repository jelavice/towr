/*
 * wheel_direction_constraint.h
 *
 *  Created on: Jul 26, 2018
 *      Author: jelavice
 */

#pragma once

#include <ifopt/constraint_set.h>

#include <towr/variables/nodes_variables_phase_based.h>
#include <towr/terrain/height_map.h> // for friction cone

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>

namespace towr {

/**
 * @brief Ensures foot force that is unilateral and inside friction cone.
 *
 * This class is responsible for constraining the endeffector xyz-forces to
 * only push into the terrain and additionally stay inside the friction cone
 * according to the current slope.
 *
 * In order to keep the constraint linear and simple for the solver to solve,
 * we approximate the friction cone by a 4-sided pyramid.
 *
 * Attention: Constraint is enforced only at the spline nodes. In between
 * violations of this constraint can occur.
 *
 * @ingroup Constraints
 */
class WheelDirectionConstraint : public ifopt::ConstraintSet
{
 public:
  using Vector3d = Eigen::Vector3d;
  using EE = uint;

  WheelDirectionConstraint(EE endeffector_id, const SplineHolder *spline_holder);
  virtual ~WheelDirectionConstraint() = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, Jacobian&) const override;

 private:
  NodesVariablesPhaseBased::Ptr ee_wheel_angles_;  ///< the current xyz foot forces.
  NodesVariablesPhaseBased::Ptr ee_motion_;  ///< the current xyz foot positions.
  EE ee_;

  const SplineHolder *spline_holder_;

  int n_constraints_per_node_;  ///< number of constraint for each node.

  //todo move this to parameters class
  const double max_turning_angle_ = 40.0 * M_PI / 180.0;
  const double max_velocity_ = 2.0; // m/s
  const double max_turning_rate_ = 10 * M_PI / 180.0; // rad/s

  /**
   * The are those Hermite-nodes that shape the polynomial during the
   * stance phases, while all the others are already set to zero force (swing)
   **/
  std::vector<int> stance_nodes_ids_;
};

} /* namespace towr */

