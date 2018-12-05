/*
 * parameters_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/parameters.h"
#include <iostream>

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

  void PrintAllParams();

  void AddJointVariables();
  void ClearAllVariables();

  //constraints
  void SetTerrainConstraint();
  void DeleteAllConstraints();
  void SetJointLimitsconstraint();
  void SetSwingConstraint() override final;

//set all the params
  void SetJointPolynomialDuration(double dt);
  void SetJointLimitsConstraintDt(double dt);
  void SetNormalFoceLimit(double fmax);
  void SetDynamicConstraintDt(double dt);
  void SetBasePolynomialDuration(double dt);
  void SetRangeOfMotionConstraintDt(double dt);

  //other useful or not so useful shit
  VecTimes GetJointPolyDurations() const;
  int GetEECount() const override final;

  std::vector<int> bounds_initial_lin_pos, bounds_initial_lin_vel, bounds_initial_ang_pos,
      bounds_initial_ang_vel;

  std::vector<bool> use_bounds_initial_ee_pos;

 private:

  VecTimes GetAnyPolyDurations(double polynomial_duration) const;

  int n_ee_;

  double dt_joint_limit_constraint_;
  double duration_joint_polynomials_;

  std::vector<VariableSetName> variables_used_;

};



} /* namespace */
