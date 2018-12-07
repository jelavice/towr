/*
 * nlp_formulation_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/nlp_formulation_extended.h"
#include "towr/variables/nodes_variables_ee_joints.h"
#include "towr/variables/nodes_variables_ee_motion_with_wheels.h"

#include "towr/variables/variable_names.h"
#include "towr/constraints/ee_joint_limits_constraint.h"
#include "towr/constraints/ee_forward_kinematics_constraint.h"
#include "towr/constraints/ee_motion_with_wheels_constraint.h"
#include "towr/constraints/terrain_constraint.h"

namespace towr {

using VariablePtrVec = NlpFormulationExtended::VariablePtrVec;
using ConstraintPtrVec = NlpFormulationExtended::ConstraintPtrVec;

void NlpFormulationExtended::CastPointers(KinematicModelWithJoints::Ptr *model,
                                          ParametersExtended::Ptr *params) const
{
  //hack
  *model = std::dynamic_pointer_cast<KinematicModelWithJoints>(model_.kinematic_model_);

  if (model == nullptr)
    throw std::runtime_error("Could not cast kinematicModelPtr to KinematicModelWithJoints");

  *params = std::dynamic_pointer_cast<ParametersExtended>(params_);

  if (params == nullptr)
    throw std::runtime_error("Could not cast ParametersPtr to ExtendedParametersPtr");
}

std::vector<NodesVariables::Ptr> NlpFormulationExtended::MakeEEMotionWithWheelsVariables() const
{

  KinematicModelWithJoints::Ptr model;
  ParametersExtended::Ptr params;
  CastPointers(&model, &params);

  std::vector<NodesVariables::Ptr> vars;

  //todo implement getEEmotion with wheel duration
  int n_nodes = params->GetJointPolyDurations().size() + 1;

  for (int ee = 0; ee < params->GetEECount(); ++ee) {

    if (model->EEhasWheel(ee) == false)
      continue;

    auto ee_motion_with_wheels = std::make_shared<NodesVariablesEEMotionWithWheels>(
        n_nodes, k3D, id::EEMotionWithWheelsNodes(ee), ee);

    Eigen::VectorXd initialPosition(k3D);
    Eigen::VectorXd finalPosition(k3D);

    //todo make better initialization
    double totalTime = params->GetTotalTime();
    ee_motion_with_wheels->SetByLinearInterpolation(initialPosition, finalPosition, totalTime);
    vars.push_back(ee_motion_with_wheels);
  }

  return vars;
}

std::vector<NodesVariables::Ptr> NlpFormulationExtended::MakeJointVariables() const
{

  auto extended_params = params_->as<Params>();

  std::vector<NodesVariables::Ptr> vars;
  int n_nodes = extended_params->GetJointPolyDurations().size() + 1;

  for (int ee = 0; ee < extended_params->GetEECount(); ++ee) {

    int numDof = model_.kinematic_model_->as<KinematicModelWithJoints>()->GetNumDof(ee);
    auto joint_spline = std::make_shared<NodesVariablesEEJoints>(n_nodes, numDof,
                                                                 id::EEJointNodes(ee), ee);
    Eigen::VectorXd initial_joint_pos(numDof);
    Eigen::VectorXd final_joint_pos(numDof);
    for (int j = 0; j < numDof; ++j) {
      double position = 0.0;
      initial_joint_pos(j) = position;
      final_joint_pos(j) = position;
    }
    double totalTime = extended_params->GetTotalTime();
    joint_spline->SetByLinearInterpolation(initial_joint_pos, final_joint_pos, totalTime);
    vars.push_back(joint_spline);
  }

  return vars;
}

std::vector<NodesVariables::Ptr> NlpFormulationExtended::MakeBaseVariables() const
{

  std::vector<NodesVariables::Ptr> vars;

  auto extended_params = params_->as<Params>();

  int n_nodes = extended_params->GetBasePolyDurations().size() + 1;

  auto spline_lin = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes);

  double x = final_base_.lin.p().x();
  double y = final_base_.lin.p().y();
  double z = terrain_->GetHeight(x, y)
      - model_.kinematic_model_->GetNominalStanceInBase().front().z();
  Vector3d final_pos(x, y, z);

  double totalTime = extended_params->GetTotalTime();
  spline_lin->SetByLinearInterpolation(initial_base_.lin.p(), final_pos, totalTime);
  spline_lin->AddStartBound(kPos, extended_params->bounds_initial_lin_pos, initial_base_.lin.p());
  spline_lin->AddStartBound(kVel, extended_params->bounds_initial_lin_vel, initial_base_.lin.v());
  spline_lin->AddFinalBound(kPos, extended_params->bounds_final_lin_pos, final_base_.lin.p());
  spline_lin->AddFinalBound(kVel, extended_params->bounds_final_lin_vel, final_base_.lin.v());
  vars.push_back(spline_lin);

  auto spline_ang = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_ang_nodes);
  spline_ang->SetByLinearInterpolation(initial_base_.ang.p(), final_base_.ang.p(), totalTime);
  spline_ang->AddStartBound(kPos, extended_params->bounds_initial_ang_pos, initial_base_.ang.p());
  spline_ang->AddStartBound(kVel, extended_params->bounds_initial_ang_vel, initial_base_.ang.v());
  spline_ang->AddFinalBound(kPos, extended_params->bounds_final_ang_pos, final_base_.ang.p());
  spline_ang->AddFinalBound(kVel, extended_params->bounds_final_ang_vel, final_base_.ang.v());
  vars.push_back(spline_ang);

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr> NlpFormulationExtended::MakeEndeffectorVariables() const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  KinematicModelWithJoints::Ptr model;
  ParametersExtended::Ptr params;
  CastPointers(&model, &params);
  ;

  // Endeffector Motions
  double T = params->GetTotalTime();
  for (int ee = 0; ee < params->GetEECount(); ee++) {

    if (model->EEhasWheel(ee))  //this is only for non wheels
      continue;

    auto nodes = std::make_shared<NodesVariablesEEMotion>(params->GetPhaseCount(ee),
                                                          params->ee_in_contact_at_start_.at(ee),
                                                          id::EEMotionNodes(ee),
                                                          params->ee_polynomials_per_swing_phase_);

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

    if (params->use_bounds_initial_ee_pos.at(ee))
      nodes->AddStartBound(kPos, { X, Y, Z }, initial_ee_W_.at(ee));
    vars.push_back(nodes);
  }

  return vars;
}

VariablePtrVec NlpFormulationExtended::GetVariableSets(SplineHolder& spline_holder)
{
  //ugly
  auto casted_ptr = dynamic_cast<SplineContainer*>(spline_holder.as<SplineHolder>());
  if (casted_ptr == nullptr)
    throw std::runtime_error("the object passed inside is not the Spline container type");

  VariablePtrVec vars;

  auto params = params_->as<ParametersExtended>();

  for (auto name : params->variables_used_)
    CreateVariableSet(name, spline_holder, vars);

  return vars;

}

void NlpFormulationExtended::CreateVariableSet(Params::VariableSetName var_set, SplineHolder &s,
                                               VariablePtrVec &vars)
{

  auto spline_holder = s.as<SplineContainer>();
  auto params = params_->as<Params>();

  switch (var_set) {
    case Params::BaseVariables: {
      auto base_motion = MakeBaseVariables();
      vars.insert(vars.end(), base_motion.begin(), base_motion.end());
      spline_holder->InitializeBaseLinMotion(base_motion.at(0), params->GetBasePolyDurations());  // linear
      spline_holder->InitializeBaseAngMotion(base_motion.at(1), params->GetBasePolyDurations());  // ang
      break;
    }

    case Params::EEMotionVariables: {
      auto ee_motion = MakeEndeffectorVariables();
      vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());
      spline_holder->InitializeEEMotion(ee_motion, params->IsOptimizeTimings());
      break;
    }

    case Params::ContactForceVariables: {
      auto ee_force = MakeForceVariables();
      vars.insert(vars.end(), ee_force.begin(), ee_force.end());
      spline_holder->InitializeEEForce(ee_force, params->IsOptimizeTimings());
      break;
    }

    case Params::ContactScheduleVariables: {
      auto contact_schedule = MakeContactScheduleVariables();
      // can also just be fixed timings that aren't optimized over, but still added
      // to spline_holder.
      if (params->IsOptimizeTimings()) {
        vars.insert(vars.end(), contact_schedule.begin(), contact_schedule.end());
      }
      spline_holder->InitializePhaseDurations(contact_schedule);
      break;
    }

    case Params::JointVariables: {
      auto ee_joint_motion = MakeJointVariables();
      vars.insert(vars.end(), ee_joint_motion.begin(), ee_joint_motion.end());
      spline_holder->InitializeJointMotion(ee_joint_motion, params->GetJointPolyDurations());
      break;
    }

    case Params::EEMotionWithWheelsVariables: {
      auto ee_wheel_motion = MakeEEMotionWithWheelsVariables();
      vars.insert(vars.end(), ee_wheel_motion.begin(), ee_wheel_motion.end());
      spline_holder->InitializeEEMotionWithWheels(ee_wheel_motion, params->GetJointPolyDurations());
      break;
    }
  }

}

ConstraintPtrVec NlpFormulationExtended::GetConstraint(Parameters::ConstraintName name,
                                                       const SplineHolder& s) const
{

  //okay first checkt the stuff that I have newly added

  switch (name) {
    case Parameters::JointLimits:
      return MakeJointLimitsConstraint(s);
    case Parameters::ForwardKinematics:
      return MakeForwardKinematicsConstraint(s);
    case Parameters::EEMotionWithWheels:
      return MakeEEMotionWithWheelsConstraint(s);
  }

  //now check the stuff that has been there from before
  return Base::GetConstraint(name, s);

}

ConstraintPtrVec NlpFormulationExtended::MakeJointLimitsConstraint(const SplineHolder &s) const
{

  ConstraintPtrVec c;

  KinematicModelWithJoints::Ptr model;
  ParametersExtended::Ptr params;
  CastPointers(&model, &params);

  for (int ee = 0; ee < params->GetEECount(); ++ee) {
    auto joint_con = std::make_shared<EEjointLimitsConstraint>(model, params->GetTotalTime(),
                                                               params->dt_joint_limit_constraint_,
                                                               ee, s);
    c.push_back(joint_con);
  }

  return c;
}

ConstraintPtrVec NlpFormulationExtended::MakeForwardKinematicsConstraint(
    const SplineHolder &s) const
{
  ConstraintPtrVec c;

  KinematicModelWithJoints::Ptr model;
  ParametersExtended::Ptr params;
  CastPointers(&model, &params);

  for (int ee = 0; ee < params->GetEECount(); ++ee) {

    double totalTime = params->GetTotalTime();
    auto fwdKinematicsCon = std::make_shared<EEforwardKinematicsConstraint>(
        model, totalTime, params->dt_forward_kinematics_constraint_, ee, s);
    c.push_back(fwdKinematicsCon);

  }

  return c;
}

ConstraintPtrVec NlpFormulationExtended::MakeEEMotionWithWheelsConstraint(
    const SplineHolder &s) const
{

  ConstraintPtrVec c;

  KinematicModelWithJoints::Ptr model;
  ParametersExtended::Ptr params;
  CastPointers(&model, &params);

  for (int ee = 0; ee < params->GetEECount(); ++ee) {
    auto con = std::make_shared<EEMotionWithWheelsConstraint>(model, params->GetTotalTime(),
                                                              params->dt_joint_limit_constraint_,
                                                              ee, s);
    c.push_back(con);
  }

  return c;
}

ConstraintPtrVec NlpFormulationExtended::MakeTerrainConstraint() const
{
  ConstraintPtrVec constraints;

  KinematicModelWithJoints::Ptr model;
  ParametersExtended::Ptr params;
  CastPointers(&model, &params);

  for (int ee = 0; ee < params->GetEECount(); ee++) {

    std::string constraintName;
    if (model->EEhasWheel(ee))
      constraintName = id::EEMotionWithWheelsNodes(ee);
    else
      constraintName = id::EEMotionNodes(ee);

    auto c = std::make_shared<TerrainConstraint>(terrain_, constraintName);
    constraints.push_back(c);

  }

  return constraints;
}

} /* namespace */

