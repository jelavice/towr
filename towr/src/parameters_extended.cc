/*
 * parameters_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/parameters_extended.h"
#include <iostream>

using VecTimes = towr::ParametersExtended::Base::VecTimes;

namespace towr {

ParametersExtended::ParametersExtended(int n_ee)
    : Parameters()
{
  n_ee_ = n_ee;

  DeleteAllConstraints();  //base class will have set buncha crap already that might be redundant

  //we need those by default
  AddContactScheduleVariables();

}
//variables

void ParametersExtended::AddContactForceVariables()
{
  variables_used_.push_back(ContactForceVariables);
}

void ParametersExtended::AddBaseVariables()
{
  variables_used_.push_back(BaseVariables);
}

void ParametersExtended::AddContactScheduleVariables()
{
  variables_used_.push_back(ContactScheduleVariables);
}

void ParametersExtended::AddJointVariables()
{
  variables_used_.push_back(JointVariables);
}

void ParametersExtended::AddEEMotionVariables(){
  variables_used_.push_back(EEMotionVariables);
}

void ParametersExtended::ClearAllVariables()
{
  variables_used_.clear();
}

// constraints

void ParametersExtended::SetTerrainConstraint()
{
  constraints_.push_back(Terrain);
}

void ParametersExtended::DeleteAllConstraints()
{
  constraints_.clear();
}

void ParametersExtended::SetJointVelocityAndPositionLimitConstraint()
{
  constraints_.push_back(JointVelocityAndPositionLimits);
}

void ParametersExtended::SetSwingConstraint() {

  std::cout << "\n====== WARNING ==============" << std::endl;
  std::cout << "=============================" << std::endl;
  std::cout <<  "If you have an end-effector that is not in contact at the beginning, swing constraint might segfault in the ipopt" << std::endl;

  constraints_.push_back(Swing);

}

//parameters

void ParametersExtended::SetJointPolynomialDuration(double dt)
{
  duration_joint_polynomials_ = dt;
}

void ParametersExtended::SetJointVelocityAndPositionLimitConstraintDt(double dt)
{
  dt_joint_velocity_and_position_limit_constraint_ = dt;
}

void ParametersExtended::SetDynamicConstraintDt(double dt)
{
  dt_constraint_dynamic_ = dt;
}

void ParametersExtended::SetBasePolynomialDuration(double dt)
{
  duration_base_polynomial_ = dt;
}

void ParametersExtended::SetNormalFoceLimit(double fmax)
{
  force_limit_in_normal_direction_ = fmax;
}

void ParametersExtended::SetRangeOfMotionConstraintDt(double dt)
{
  dt_constraint_range_of_motion_ = dt;
}

// other methods

VecTimes ParametersExtended::GetAnyPolyDurations(double polynomial_duration) const
{

  std::vector<double> any_spline_timings_;
  double dt = polynomial_duration;
  double t_left = GetTotalTime();

  double eps = 1e-10;  // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left > dt ? dt : t_left;
    any_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return any_spline_timings_;
}

VecTimes ParametersExtended::GetJointPolyDurations() const
{
  return GetAnyPolyDurations(duration_joint_polynomials_);
}

int ParametersExtended::GetEECount() const
{
  return n_ee_;
}

} /* namespace*/

