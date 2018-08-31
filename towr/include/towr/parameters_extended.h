/*
 * parameters_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/parameters.h"

namespace towr {

class ParametersExtended : public Parameters
{

 public:

  using Base = Parameters;

  enum ExtendedConstraintSet
  {
  //thse enums are deifined in the base class
  //todo fix this
  };

  ParametersExtended() = delete;
  ParametersExtended(int n_ee);
  ~ParametersExtended() = default;

  int GetEECount() const final;

  void MakeTerrainConstraint();

  void DeleteAllConstraints();

  void SetJointVelocityAndPositionLimitConstraint();

//set all the params
  void SetJointPolynomialDuration(double dt);

  void SetJointVelocityAndPositionLimitConstraintDt(double dt);

  void SetNormalFoceLimit(double fmax);

  void SetDynamicConstraintDt(double dt);

  void SetBasePolynomialDuration(double dt);

  void SetRangeOfMotionConstraintDt(double dt);

  VecTimes GetJointPolyDurations() const;

  VecTimes GetBasePolyDurations() const;

 private:

  VecTimes GetAnyPolyDurations(double polynomial_duration) const;

  int n_ee_;

  double dt_joint_velocity_and_position_limit_constraint_;
  double duration_joint_polynomials_;

};

} /* namespace */
