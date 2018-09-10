/*
 * nlp_formulation_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/nlp_formulation_extended.h"
#include "towr/models/kinematic_model_with_joints.h"
#include "towr/variables/nodes_variables_ee_joints.h"
#include "towr/variables/variable_names.h"

namespace towr {

using VariablePtrVec = NlpFormulationExtended::VariablePtrVec;

//NlpFormulationExtended::NlpFormulationExtended()
//    : Base()
//{
//
//}

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
    joint_spline->SetByLinearInterpolation(initial_joint_pos, final_joint_pos,
                                           extended_params->GetTotalTime());
    vars.push_back(joint_spline);
  }
  return vars;
}

std::vector<NodesVariables::Ptr> NlpFormulationExtended::MakeBaseVariables() const {

std::vector<NodesVariables::Ptr> vars;

auto extended_params = params_->as<Params>();

  int n_nodes = extended_params->GetBasePolyDurations().size() + 1;

  auto spline_lin = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes);

  double x = final_base_.lin.p().x();
  double y = final_base_.lin.p().y();
  double z = terrain_->GetHeight(x,y) - model_.kinematic_model_->GetNominalStanceInBase().front().z();
  Vector3d final_pos(x, y, z);

  spline_lin->SetByLinearInterpolation(initial_base_.lin.p(), final_pos, extended_params->GetTotalTime());
  spline_lin->AddStartBound(kPos, extended_params->bounds_initial_lin_pos, initial_base_.lin.p());
  spline_lin->AddStartBound(kVel, extended_params->bounds_initial_lin_vel, initial_base_.lin.v());
  spline_lin->AddFinalBound(kPos, extended_params->bounds_final_lin_pos,   final_base_.lin.p());
  spline_lin->AddFinalBound(kVel, extended_params->bounds_final_lin_vel, final_base_.lin.v());
  vars.push_back(spline_lin);

  auto spline_ang = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_ang_nodes);
  spline_ang->SetByLinearInterpolation(initial_base_.ang.p(), final_base_.ang.p(), extended_params->GetTotalTime());
  spline_ang->AddStartBound(kPos, extended_params->bounds_initial_ang_pos, initial_base_.ang.p());
  spline_ang->AddStartBound(kVel, extended_params->bounds_initial_ang_vel, initial_base_.ang.v());
  spline_ang->AddFinalBound(kPos, extended_params->bounds_final_ang_pos, final_base_.ang.p());
  spline_ang->AddFinalBound(kVel, extended_params->bounds_final_ang_vel, final_base_.ang.v());
  vars.push_back(spline_ang);

  return vars;
}




VariablePtrVec NlpFormulationExtended::GetVariableSets(SplineHolder& spline_holder)
{
  //ugly
  auto casted_ptr = dynamic_cast<SplineContainer*>(spline_holder.as<SplineHolder>());
  if (casted_ptr == nullptr)
    throw std::runtime_error("the object passed inside is not the Spline container type");

  VariablePtrVec vars;

  auto params = params_->as<Params>();

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
  }

}

} /* namespace */

