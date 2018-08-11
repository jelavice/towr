/******************************************************************************
 Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <towr/parameters.h>
#include <towr/variables/cartesian_dimensions.h>

#include <algorithm>
#include <numeric>      // std::accumulate
#include <math.h>       // fabs
#include <cassert>
#include <iostream>

namespace towr {

bool Parameters::robot_has_wheels_ = false;
bool Parameters::use_joint_formulation_ = false;

Parameters::Parameters()
{
  // optimization variables
  duration_base_polynomial_ = 0.1;  //affect number of variables

  ee_polynomials_per_swing_phase_ = 2;
  force_polynomials_per_stance_phase_ = 3;

  dt_constraint_range_of_motion_ = 0.08;  //anymal
  dt_constraint_dynamic_ = 0.1;

  // all the shit for the anymal
  //force_limit_in_normal_direction_ = 1000.0; //for the anymal
  force_limit_in_normal_direction_ = 30000.0;  //for the excavator

  // these are the basic constraints that always have to be set
  SetConstraints();

  bounds_final_lin_pos = {X,Y};
  bounds_final_lin_vel = {X,Y,Z};
  bounds_final_ang_pos = {X,Y,Z};
  bounds_final_ang_vel = {X,Y,Z};
  // additional restrictions are set directly on the variables in nlp_factory,
  // such as e.g. initial and endeffector,...
}

void Parameters::SetNumberEEPolynomials(int n)
{
  //todo fix this such that we don't have to have the the same number of polynomials
  force_polynomials_per_stance_phase_ = ee_polynomials_per_swing_phase_ =
      ee_polynomials_per_phase_ = n;
}

void Parameters::SetDynamicConstraintDt(double dt)
{
  dt_constraint_dynamic_ = dt;
}

void Parameters::SetRangeOfMotionConstraintDt(double dt)
{
  dt_constraint_range_of_motion_ = dt;
}

void Parameters::SetPolynomialDurationBase(double dt)
{
  duration_base_polynomial_ = dt;
}

void Parameters::SetConstraints()
{
  // first clear the shit from the constructor
  constraints_.clear();

  //now reset all the constraints with new quentities
  constraints_.push_back(Terrain);
  SetDynamicConstraint();
  SetForceConstraint();


  if (robot_has_wheels_ && use_joint_formulation_) {
    //set rom joints and ee with wheels
    //SetWheelConstraint(); //dis hacly since I am building thos constraints in the joint constraints anyway
    SetKinematicConstraintJoints();
//    std::cout << "Added wheel and joint rom constraint" << std::endl;
  }else if ((robot_has_wheels_ == false) && use_joint_formulation_){
    //set rom with joints
    SetKinematicConstraintJoints();
//    std::cout << "Added joint rom constraint" << std::endl;
  } else if ( robot_has_wheels_ && (use_joint_formulation_ == false)){
    //set wheel heading and rom
    SetWheelConstraint();
    SetKinematicConstraint();
//    std::cout << "Added wheel and rom constraint" << std::endl;
  } else
  {
    // set rom
    SetKinematicConstraint();
//    std::cout << "Added rom constraint only" << std::endl;
  }

}

void Parameters::SetDynamicConstraint()
{
  constraints_.push_back(Dynamic);
  constraints_.push_back(BaseAcc);  // so accelerations don't jump between polynomials
}

void Parameters::SetKinematicConstraint()
{
  constraints_.push_back(EndeffectorRom);
}

void Parameters::SetKinematicConstraintJoints()
{
  constraints_.push_back(EndeffectorRomJoints);
}

void Parameters::SetJointConstraint()
{
  constraints_.push_back(EndeffectorRomJoints);
}

void Parameters::SetForceConstraint()
{
  constraints_.push_back(Force);
}

void Parameters::SetWheelConstraint()
{
  constraints_.push_back(WheelHeading);
}

void Parameters::SetSwingConstraint()
{
  constraints_.push_back(Swing);
}

void Parameters::OptimizePhaseDurations()
{
  // limiting this range can help convergence when optimizing gait
  // if phase durations too short, can also cause kinematic constraint to
  // be violated, so dt_constraint_range_of_motion must be decreased.
  bound_phase_duration_.front() = 0.2;
  bound_phase_duration_.back() = 1.0;
  constraints_.push_back(TotalTime);
}

void Parameters::RestrictBaseRangeOfMotion()
{
  dt_constraint_base_motion_ = duration_base_polynomial_ / 4.;
  constraints_.push_back(BaseRom);
}

void Parameters::PenalizeEndeffectorForces()
{
  // cost weighed by 1.0
  costs_.push_back( { ForcesCostID, 1.0 });
}

Parameters::VecTimes Parameters::GetBasePolyDurations() const
{
  std::vector<double> base_spline_timings_;
  double dt = duration_base_polynomial_;
  double t_left = GetTotalTime();

  double eps = 1e-10;  // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left > dt ? dt : t_left;
    base_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return base_spline_timings_;
}

int Parameters::GetPhaseCount(EEID ee) const
{
  return ee_phase_durations_.at(ee).size();
}

int Parameters::GetEECount() const
{
  return ee_in_contact_at_start_.size();
}

double Parameters::GetTotalTime() const
{
  std::vector<double> T_feet;

  for (const auto& v : ee_phase_durations_)
    T_feet.push_back(std::accumulate(v.begin(), v.end(), 0.0));

  // safety check that all feet durations sum to same value
  double T = T_feet.empty() ? 0.0 : T_feet.front();  // take first foot as reference
  for (double Tf : T_feet)
    assert(fabs(Tf - T) < 1e-6);

  return T;
}

bool Parameters::IsOptimizeTimings() const
{
  // if total time is constrained, then timings are optimized
  ConstraintName c = TotalTime;
  auto v = constraints_;  // shorthand
  return std::find(v.begin(), v.end(), c) != v.end();
}

std::array<double, 2> Parameters::GetPhaseDurationBounds() const
{
  // adjust bound to always be less than total duration of trajectory
  double upper_bound = bound_phase_duration_.back();
  double max = GetTotalTime() > upper_bound ? upper_bound : GetTotalTime();
  return {bound_phase_duration_.front(), max};
}

}  // namespace towr
