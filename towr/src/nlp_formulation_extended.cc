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

NlpFormulationExtended::NlpFormulationExtended()
    : Base()
{

  // maybe I need craps here

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
    joint_spline->SetByLinearInterpolation(initial_joint_pos, final_joint_pos,
                                           extended_params->GetTotalTime());
    vars.push_back(joint_spline);
  }
  return vars;
}

VariablePtrVec NlpFormulationExtended::GetVariableSets(SplineHolder& spline_holder) {

  VariablePtrVec vars;

  //todo add factory here so that later I can choose which variable set I want to create


  return vars;


}


} /* namespace */

