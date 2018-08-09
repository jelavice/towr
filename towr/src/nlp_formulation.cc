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

#include <towr/nlp_formulation.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/phase_durations.h>

#include <towr/constraints/base_motion_constraint.h>
#include <towr/constraints/dynamic_constraint.h>
#include <towr/constraints/force_constraint.h>
#include <towr/constraints/range_of_motion_constraint.h>
#include <towr/constraints/swing_constraint.h>
#include <towr/constraints/terrain_constraint.h>
#include <towr/constraints/total_duration_constraint.h>
#include <towr/constraints/spline_acc_constraint.h>
#include <towr/constraints/wheel_direction_constraint.h>
#include <towr/constraints/range_of_motion_constraint_joints.h>

#include <towr/costs/node_cost.h>
#include <towr/variables/nodes_variables_all.h>
#include <towr/variables/nodes_variables_limb_joints.h>

#include <iostream>

namespace towr {

NlpFormulation::NlpFormulation()
{
  using namespace std;
  cout << "\n";
  cout << "************************************************************\n";
  cout << " TOWR - Trajectory Optimization for Walking Robots (v1.4)\n";
  cout << "                \u00a9 Alexander W. Winkler\n";
  cout << "           https://github.com/ethz-adrl/towr\n";
  cout << "************************************************************";
  cout << "\n\n";
}

NlpFormulation::VariablePtrVec NlpFormulation::GetVariableSets(SplineHolder& spline_holder)
{
  VariablePtrVec vars;

  auto base_motion = MakeBaseVariables();
  vars.insert(vars.end(), base_motion.begin(), base_motion.end());

  std::vector<NodesVariablesPhaseBased::Ptr> ee_motion;
  std::vector<NodesVariables::Ptr> joints;

  if (Parameters::use_joint_formulation_ && Parameters::use_joint_formulation_) {

    //create motion variables
    // with wheels just means that position is unconstrained during the stance phase
    ee_motion = MakeEndeffectorVariablesWithWheels();
    vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

    //create joins variables
    joints = MakeJointVariables();
    vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

  } else if (Parameters::robot_has_wheels_ && (Parameters::use_joint_formulation_ == false)) {

    //dis is what I am doing at the moment
    ee_motion = MakeEndeffectorVariablesWithWheels();
    vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

  } else if ((Parameters::robot_has_wheels_ == false) && Parameters::use_joint_formulation_) {

    // not entirely sure what to do here
    // create normal EE variables and the n also add joints
    ee_motion = MakeEndeffectorVariables();
    vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

    joints = MakeJointVariables();
    vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

  } else {

    //no wheels no joints all normal and easy
    ee_motion = MakeEndeffectorVariables();
    vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());
  }
  auto ee_force = MakeForceVariables();
  vars.insert(vars.end(), ee_force.begin(), ee_force.end());

  std::vector<NodesVariablesPhaseBased::Ptr> ee_wheels;
  if (Parameters::robot_has_wheels_) {
    ee_wheels = MakeWheelVariables();
    vars.insert(vars.end(), ee_wheels.begin(), ee_wheels.end());
  }

  auto contact_schedule = MakeContactScheduleVariables();
  // can also just be fixed timings that aren't optimized over, but still added
  // to spline_holder.
  if (params_.IsOptimizeTimings()) {
    vars.insert(vars.end(), contact_schedule.begin(), contact_schedule.end());
  }

  if (Parameters::robot_has_wheels_ && Parameters::use_joint_formulation_) {
    // call with joint variables
    spline_holder = SplineHolder(base_motion.at(0),  // linear
                                 base_motion.at(1),  // angular
                                 params_.GetBasePolyDurations(), ee_motion, ee_force, joints,
                                 contact_schedule, params_.IsOptimizeTimings());
  } else if (Parameters::robot_has_wheels_ && (Parameters::use_joint_formulation_ == false)) {
    //do what I am doing now
    spline_holder = SplineHolder(base_motion.at(0),  // linear
                                 base_motion.at(1),  // angular
                                 params_.GetBasePolyDurations(), ee_motion, ee_force, ee_wheels,
                                 contact_schedule, params_.IsOptimizeTimings());
  } else if ((Parameters::robot_has_wheels_ == false) && Parameters::use_joint_formulation_) {
    //no idea what to do here
    //I guess call again with joints
  } else {
    //do the normal thing
    spline_holder = SplineHolder(base_motion.at(0),  // linear
                                 base_motion.at(1),  // angular
                                 params_.GetBasePolyDurations(), ee_motion, ee_force,
                                 contact_schedule, params_.IsOptimizeTimings());
  }

  return vars;
}

std::vector<NodesVariables::Ptr> NlpFormulation::MakeBaseVariables() const
{
  std::vector<NodesVariables::Ptr> vars;

  int n_nodes = params_.GetBasePolyDurations().size() + 1;

  auto spline_lin = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes);

  double x = final_base_.lin.p().x();
  double y = final_base_.lin.p().y();
  double z = terrain_->GetHeight(x, y)
      - model_.kinematic_model_->GetNominalStanceInBase().front().z();
  Vector3d final_pos(x, y, z);

  spline_lin->SetByLinearInterpolation(initial_base_.lin.p(), final_pos, params_.GetTotalTime());
  spline_lin->AddStartBound(kPos, { X, Y, Z }, initial_base_.lin.p());
  spline_lin->AddStartBound(kVel, { X, Y, Z }, initial_base_.lin.v());
  spline_lin->AddFinalBound(kPos, params_.bounds_final_lin_pos, final_base_.lin.p());
  spline_lin->AddFinalBound(kVel, params_.bounds_final_lin_vel, final_base_.lin.v());
  vars.push_back(spline_lin);

  auto spline_ang = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_ang_nodes);
  spline_ang->SetByLinearInterpolation(initial_base_.ang.p(), final_base_.ang.p(),
                                       params_.GetTotalTime());
  spline_ang->AddStartBound(kPos, { X, Y, Z }, initial_base_.ang.p());
  spline_ang->AddStartBound(kVel, { X, Y, Z }, initial_base_.ang.v());
  spline_ang->AddFinalBound(kPos, params_.bounds_final_ang_pos, final_base_.ang.p());
  spline_ang->AddFinalBound(kVel, params_.bounds_final_ang_vel, final_base_.ang.v());
  vars.push_back(spline_ang);

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr> NlpFormulation::MakeEndeffectorVariables() const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  // Endeffector Motions
  double T = params_.GetTotalTime();
  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto nodes = std::make_shared<NodesVariablesEEMotion>(params_.GetPhaseCount(ee),
                                                          params_.ee_in_contact_at_start_.at(ee),
                                                          id::EEMotionNodes(ee),
                                                          params_.ee_polynomials_per_swing_phase_);

    // initialize towards final footholds
    double yaw = final_base_.ang.p().z();
    Eigen::Vector3d euler(0.0, 0.0, yaw);
    Eigen::Matrix3d w_R_b = EulerConverter::GetRotationMatrixBaseToWorld(euler);
    Vector3d final_ee_pos_W = final_base_.lin.p()
        + w_R_b * model_.kinematic_model_->GetNominalStanceInBase().at(ee);
    double x = final_ee_pos_W.x();
    double y = final_ee_pos_W.y();
    double z = terrain_->GetHeight(x, y);
    nodes->SetByLinearInterpolation(initial_ee_W_.at(ee), Vector3d(x, y, z), T);

    nodes->AddStartBound(kPos, { X, Y, Z }, initial_ee_W_.at(ee));
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr> NlpFormulation::MakeForceVariables() const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  double T = params_.GetTotalTime();
  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto nodes = std::make_shared<NodesVariablesEEForce>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee), id::EEForceNodes(ee),
        params_.force_polynomials_per_stance_phase_);

    // initialize with mass of robot distributed equally on all legs
    double m = model_.dynamic_model_->m();
    double g = model_.dynamic_model_->g();

    Vector3d f_stance(0.0, 0.0, m * g / params_.GetEECount());
    nodes->SetByLinearInterpolation(f_stance, f_stance, T);  // stay constant
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<PhaseDurations::Ptr> NlpFormulation::MakeContactScheduleVariables() const
{
  std::vector<PhaseDurations::Ptr> vars;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto var = std::make_shared<PhaseDurations>(ee, params_.ee_phase_durations_.at(ee),
                                                params_.ee_in_contact_at_start_.at(ee),
                                                params_.GetPhaseDurationBounds().front(),
                                                params_.GetPhaseDurationBounds().back());
    vars.push_back(var);
  }

  return vars;
}

//todo implement
std::vector<NodesVariables::Ptr> NlpFormulation::MakeJointVariables() const
{

  std::vector<NodesVariables::Ptr> vars;

  // I gues this is the same for the joints and the base
  //todo look into that
  int n_nodes = params_.GetBasePolyDurations().size() + 1;

  //need to iterate and make for every goddamn limb its joint variables
  for (int ee = 0; ee < model_.kinematic_model_->GetNumberOfEndeffectors(); ++ee) {

    int numDof = model_.kinematic_model_->GetNumDof(ee);
    auto joint_spline = std::make_shared<NodesVariablesLimbJoints>(n_nodes, numDof,
                                                                   id::JointNodes(ee), ee);
    Eigen::VectorXd initial_joint_pos(numDof);
    Eigen::VectorXd final_joint_pos(numDof);
    std::vector<int> dimensions;
    for (int j = 0; j < numDof; ++j) {
      initial_joint_pos(j) = 0.0;
      final_joint_pos(j) = 0.0;
      dimensions.push_back(j);
    }

    joint_spline->SetByLinearInterpolation(initial_joint_pos, final_joint_pos,
                                           params_.GetTotalTime());
    //not sure I even want these bounds, I only care about the positin of the base
//    joint_spline->AddStartBound(kPos, dimensions, initial_joint_pos);
//    joint_spline->AddStartBound(kVel, dimensions, initial_joint_pos); //initial joint velocoity is zero
//    joint_spline->AddFinalBound(kPos, params_.bounds_final_lin_pos, final_base_.lin.p());
//    joint_spline->AddFinalBound(kVel, params_.bounds_final_lin_vel, final_base_.lin.v());
    vars.push_back(joint_spline);

  }

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr> NlpFormulation::MakeWheelVariables() const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  double T = params_.GetTotalTime();

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto nodes = std::make_shared<NodesVariablesWheelAngle>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee), id::WheelAngleNodes(ee),
        params_.force_polynomials_per_stance_phase_);

    Eigen::VectorXd angle_wheel(1), angle_wheel_final(1);
    angle_wheel(0) = 0.0;

    //set it to the final  base stance
    Eigen::Vector2d heading = (final_base_.lin.p() - initial_base_.lin.p()).head(2);

    angle_wheel_final(0) = 10.0 / 180.0 * M_PI * 0.0;
    nodes->SetByLinearInterpolation(angle_wheel, angle_wheel_final, T);  // stay constant
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr> NlpFormulation::MakeEndeffectorVariablesWithWheels() const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

// Endeffector Motions
  double T = params_.GetTotalTime();
  for (int ee = 0; ee < params_.GetEECount(); ee++) {

    auto nodes = std::make_shared<NodesVariablesEEMotionWithWheels>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee), id::EEMotionNodes(ee),
        params_.ee_polynomials_per_swing_phase_);

    // initialize towards final footholds
    double yaw = final_base_.ang.p().z();
    Eigen::Vector3d euler(0.0, 0.0, yaw);
    Eigen::Matrix3d w_R_b = EulerConverter::GetRotationMatrixBaseToWorld(euler);
    Vector3d final_ee_pos_W = final_base_.lin.p()
        + w_R_b * model_.kinematic_model_->GetNominalStanceInBase().at(ee);
    double x = final_ee_pos_W.x();
    double y = final_ee_pos_W.y();
    double z = terrain_->GetHeight(x, y);
    nodes->SetByLinearInterpolation(initial_ee_W_.at(ee), Vector3d(x, y, z), T);

    nodes->AddStartBound(kPos, { X, Y, Z }, initial_ee_W_.at(ee));
    vars.push_back(nodes);
  }

  return vars;
}

NlpFormulation::ContraintPtrVec NlpFormulation::GetConstraints(
    const SplineHolder& spline_holder) const
{
  ContraintPtrVec constraints;
  for (auto name : params_.constraints_)
    for (auto c : GetConstraint(name, spline_holder))
      constraints.push_back(c);

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::GetConstraint(Parameters::ConstraintName name,
                                                              const SplineHolder& s) const
{
  switch (name) {
    case Parameters::Dynamic:
      return MakeDynamicConstraint(s);
    case Parameters::EndeffectorRom:
      return MakeRangeOfMotionBoxConstraint(s);
    case Parameters::BaseRom:
      return MakeBaseRangeOfMotionConstraint(s);
    case Parameters::TotalTime:
      return MakeTotalTimeConstraint();
    case Parameters::Terrain:
      return MakeTerrainConstraint();
    case Parameters::Force:
      return MakeForceConstraint();
    case Parameters::Swing:
      return MakeSwingConstraint();
    case Parameters::BaseAcc:
      return MakeBaseAccConstraint(s);
    case Parameters::WheelHeading:
      return MakeWheelConstraint(s);
    case Parameters::EndeffectorRomJoints:
      return MakeRangeOfMotionConstraintJoints(s);
    default:
      throw std::runtime_error("constraint not defined!");
  }
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeBaseRangeOfMotionConstraint(
    const SplineHolder& s) const
{
  return {std::make_shared<BaseMotionConstraint>(params_.GetTotalTime(),
        params_.dt_constraint_base_motion_,
        s)};
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeDynamicConstraint(const SplineHolder& s) const
{
  auto constraint = std::make_shared<DynamicConstraint>(model_.dynamic_model_,
                                                        params_.GetTotalTime(),
                                                        params_.dt_constraint_dynamic_, s);
  return {constraint};
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeRangeOfMotionBoxConstraint(
    const SplineHolder& s) const
{
  ContraintPtrVec c;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto rom = std::make_shared<RangeOfMotionConstraint>(model_.kinematic_model_,
                                                         params_.GetTotalTime(),
                                                         params_.dt_constraint_range_of_motion_, ee,
                                                         s);
    c.push_back(rom);
  }

  return c;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeTotalTimeConstraint() const
{
  ContraintPtrVec c;
  double T = params_.GetTotalTime();

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto duration_constraint = std::make_shared<TotalDurationConstraint>(T, ee);
    c.push_back(duration_constraint);
  }

  return c;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeTerrainConstraint() const
{
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto c = std::make_shared<TerrainConstraint>(terrain_, id::EEMotionNodes(ee));
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeForceConstraint() const
{
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto c = std::make_shared<ForceConstraint>(terrain_, params_.force_limit_in_normal_direction_,
                                               ee);
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeWheelConstraint(const SplineHolder& s) const
{
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto c = std::make_shared<WheelDirectionConstraint>(ee, &s);
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeRangeOfMotionConstraintJoints(
    const SplineHolder& s) const
{
  ContraintPtrVec constraints;


  //hack
  auto model_ptr = std::dynamic_pointer_cast<KinematicModelJoints>(model_.kinematic_model_);

  if (model_ptr == nullptr)
    throw std::runtime_error("Dynamic cast to KinematicModelJoints failed");

  //todo see whether here we need to pass something else
  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto c = std::make_shared<RangeOfMotionConstraintJoints>(model_ptr,
                                                             params_.GetTotalTime(),
                                                             params_.dt_constraint_range_of_motion_,
                                                             ee, s);
    constraints.push_back(c);
  }

  return constraints;
}

//todo fix the segfault that I get when I use wheels and swing constraint
NlpFormulation::ContraintPtrVec NlpFormulation::MakeSwingConstraint() const
{
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto swing = std::make_shared<SwingConstraint>(id::EEMotionNodes(ee));
    constraints.push_back(swing);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeBaseAccConstraint(const SplineHolder& s) const
{
  ContraintPtrVec constraints;

  constraints.push_back(std::make_shared<SplineAccConstraint>(s.base_linear_, id::base_lin_nodes));

  constraints.push_back(std::make_shared<SplineAccConstraint>(s.base_angular_, id::base_ang_nodes));

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::GetCosts() const
{
  ContraintPtrVec costs;
  for (const auto& pair : params_.costs_)
    for (auto c : GetCost(pair.first, pair.second))
      costs.push_back(c);

  return costs;
}

NlpFormulation::CostPtrVec NlpFormulation::GetCost(const Parameters::CostName& name,
                                                   double weight) const
{
  switch (name) {
    case Parameters::ForcesCostID:
      return MakeForcesCost(weight);
    case Parameters::EEMotionCostID:
      return MakeEEMotionCost(weight);
    default:
      throw std::runtime_error("cost not defined!");
  }
}

NlpFormulation::CostPtrVec NlpFormulation::MakeForcesCost(double weight) const
{
  CostPtrVec cost;

  for (int ee = 0; ee < params_.GetEECount(); ee++)
    cost.push_back(std::make_shared<NodeCost>(id::EEForceNodes(ee), kPos, Z));

  return cost;
}

NlpFormulation::CostPtrVec NlpFormulation::MakeEEMotionCost(double weight) const
{
  CostPtrVec cost;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    cost.push_back(std::make_shared<NodeCost>(id::EEMotionNodes(ee), kVel, X));
    cost.push_back(std::make_shared<NodeCost>(id::EEMotionNodes(ee), kVel, Y));
  }

  return cost;
}

} /* namespace towr */
