/*
 * parameters_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/parameters_extended.h"
#include <iostream>
#include <cmath>
#include "towr/variables/cartesian_dimensions.h"

using VecTimes = towr::ParametersExtended::Base::VecTimes;

namespace towr {

ParametersExtended::ParametersExtended(int n_ee)
    : Parameters()
{
  n_ee_ = n_ee;

  DeleteAllConstraints();  //base class will have set buncha crap already that might be redundant

  //we need those by default
  AddContactScheduleVariables();

  for (int i = 0; i < n_ee; ++i)
    use_bounds_initial_ee_pos.push_back(true); // add initial bounds_by default for the ee's

  //set by default all bounds to be active
  bounds_initial_ang_pos = {X,Y,Z};
  bounds_initial_ang_vel = {X,Y,Z};
  bounds_initial_lin_pos = {X,Y,Z};
  bounds_initial_lin_vel = {X,Y,Z};

  SetJointPolynomialDuration(0.02);
  SetJointVelocityAndPositionLimitConstraintDt(0.1);

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
  while (t_left > eps && t_left >= 0.0) {
    double duration = t_left > dt ? dt : t_left;
    any_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return any_spline_timings_;
}

VecTimes ParametersExtended::GetJointPolyDurations() const
{
  const double eps = 1e-6;
  if (std::fabs(duration_joint_polynomials_) <= eps)
    std::cerr << "Joint polynomial dt not set" << std::endl;

  return GetAnyPolyDurations(duration_joint_polynomials_);
}

int ParametersExtended::GetEECount() const
{
  return n_ee_;
}


template<typename BasicType>
void PrintVectors(const std::vector<BasicType> &vec, const std::string &name)
{

  std::cout << "Printing " << name << " , elements: ";

  for (int i = 0; i < vec.size(); ++i) {
    std::cout << vec.at(i) << ", ";
  }

  std::cout << std::endl;

}


void ParametersExtended::PrintAllParams(){

  std::cout << "STUFF FROM EXTENDED PARAMS: " << std::endl;
  std::cout << "Num endeffectors: " << n_ee_ << std::endl;
  std::cout << "Duration of joint polynomials: " << duration_joint_polynomials_ << std::endl;
  std::cout << "Joint velocity and position limit constraint dt: " << dt_joint_velocity_and_position_limit_constraint_ << std::endl;
  PrintVectors<VariableSetName>(variables_used_, "variables used");
  PrintVectors<int>(bounds_initial_lin_pos, "bounds_initial_lin_pos");
  PrintVectors<int>(bounds_initial_lin_vel, "bounds_initial_lin_vel");
  PrintVectors<int>(bounds_initial_ang_pos, "bounds_initial_ang_pos");
  PrintVectors<int>(bounds_initial_ang_vel, "bounds_initial_ang_vel");
  PrintVectors<bool>(use_bounds_initial_ee_pos, "use_bounds_initial_ee_pos");

  std::cout << "STUFF FROM NORMAL PARAMS: " << std::endl;
  std::cout << "dt constraint dynamic: " << dt_constraint_dynamic_ << std::endl;
  std::cout <<  "dt constraint range of motion: " << dt_constraint_range_of_motion_ << std::endl;
  std::cout << "dt constraint base motion: " << dt_constraint_base_motion_ << std::endl;
  std::cout << "Duration base polynomial: " << duration_base_polynomial_ << std::endl;
  std::cout << "ee polynomials per swing phase: " << ee_polynomials_per_swing_phase_ << std::endl;
  std::cout << "force polynomials per stance phase: " << force_polynomials_per_stance_phase_
            << std::endl;
  std::cout << "force limit in normal direction: " << force_limit_in_normal_direction_ << std::endl;
  PrintVectors<int>(bounds_final_lin_pos, "bounds_final_lin_pos");
  PrintVectors<int>(bounds_final_lin_vel, "bounds_final_lin_vel");
  PrintVectors<int>(bounds_final_ang_pos, "bounds_final_ang_pos");
  PrintVectors<int>(bounds_final_ang_vel, "bounds_final_ang_vel");
  PrintVectors<ConstraintName>(constraints_, "used constraints");
  std::cout << "Total time: " << GetTotalTime() << std::endl;

}


} /* namespace*/

