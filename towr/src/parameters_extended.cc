/*
 * parameters_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/parameters_extended.h"

using VecTimes = towr::ParametersExtended::Base::VecTimes;

namespace towr {

ParametersExtended::ParametersExtended(int n_ee)
    : Parameters()
{
  n_ee_ = n_ee;

  DeleteAllConstraints();  //base class will have set buncha crap already that might be redundant
}

int ParametersExtended::GetEECount() const
{
  return n_ee_;
}

void ParametersExtended::MakeTerrainConstraint()
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

VecTimes ParametersExtended::GetAnyPolyDurations(double polynomial_duration) const
{

  std::vector<double> any_spline_timings_;
  double dt = polynomial_duration;
  double t_left = GetTotalTime();  //todo check this

  double eps = 1e-10;  // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left > dt ? dt : t_left;
    any_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return any_spline_timings_;
}

VecTimes ParametersExtended::GetBasePolyDurations() const
{

  return GetAnyPolyDurations(duration_base_polynomial_);
}

VecTimes ParametersExtended::GetJointPolyDurations() const
{

  return GetAnyPolyDurations(duration_joint_polynomials_);

}



} /* namespace*/

