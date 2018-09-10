/*
 * parameters_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/parameters.h"

namespace towr {

struct ParametersExtended : public Parameters
{
  using Ptr = std::shared_ptr<ParametersExtended>;
  using Base = Parameters;

  enum VariableSetName
  {
    BaseVariables,
    EEMotionVariables,
    ContactForceVariables,
    ContactScheduleVariables,  //added by default
    JointVariables
  };

  enum ExtendedConstraintSet
  {
  //thse enums are deifined in the base class
  //todo fix this
  };

  friend class NlpFormulationExtended;

  ParametersExtended() = delete;
  ParametersExtended(int n_ee);
  ~ParametersExtended() = default;

  //variables
  void AddBaseVariables();
  void AddEEMotionVariables();
  void AddContactForceVariables();

 private:
  void AddContactScheduleVariables();

 public:
  void AddJointVariables();
  void ClearAllVariables();

  //constraints
  void SetTerrainConstraint();
  void DeleteAllConstraints();
  void SetJointVelocityAndPositionLimitConstraint();
  void SetSwingConstraint() override final;

//set all the params
  void SetJointPolynomialDuration(double dt);
  void SetJointVelocityAndPositionLimitConstraintDt(double dt);
  void SetNormalFoceLimit(double fmax);
  void SetDynamicConstraintDt(double dt);
  void SetBasePolynomialDuration(double dt);
  void SetRangeOfMotionConstraintDt(double dt);

  //other useful or not so useful shit
  VecTimes GetJointPolyDurations() const;
  int GetEECount() const override final;

  std::vector<int> bounds_initial_lin_pos, bounds_initial_lin_vel, bounds_initial_ang_pos,
      bounds_initial_ang_vel;

  std::vector<int> bounds_final_lin_pos, bounds_final_lin_vel, bounds_final_ang_pos,
      bounds_final_ang_vel;

 private:

  VecTimes GetAnyPolyDurations(double polynomial_duration) const;

  int n_ee_;

  double dt_joint_velocity_and_position_limit_constraint_;
  double duration_joint_polynomials_;

  std::vector<VariableSetName> variables_used_;

};

} /* namespace */
