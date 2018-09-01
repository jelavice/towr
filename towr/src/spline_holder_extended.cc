/*
 * spline_holder_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/variables/spline_holder_extended.h"
#include <memory>
#include <towr/variables/phase_spline.h>

namespace towr {

SplineHolderExtended::SplineHolderExtended(NodesVariables::Ptr base_lin,
                                           NodesVariables::Ptr base_ang,
                                           const std::vector<double>& base_poly_durations,
                                           std::vector<NodesVariablesPhaseBased::Ptr> ee_motion,
                                           std::vector<NodesVariablesPhaseBased::Ptr> ee_force,
                                           std::vector<PhaseDurations::Ptr> phase_durations,
                                           bool ee_durations_change)
    : Base(base_lin, base_ang, base_poly_durations, ee_motion, ee_force, phase_durations,
           ee_durations_change)
{

}

void SplineHolderExtended::InitializePhaseBasedDurations(std::vector<NodesVariablesPhaseBased::Ptr> nodes_phase_based,
                                   bool durations_change, std::vector<NodeSpline::Ptr> &node_spline)
{


  if (phase_durations_initialized_ == false){
    throw std::runtime_error("Phase durations need to be initialized first");
  }

  for (uint ee = 0; ee < nodes_phase_based.size(); ++ee) {
    if (durations_change) {
      // spline that changes the polynomial durations (affects Jacobian)
      node_spline.push_back(
          std::make_shared<PhaseSpline>(nodes_phase_based.at(ee), phase_durations_.at(ee).get()));
    } else {
      // spline without changing the polynomial durations
      auto ee_phase_poly_durations = nodes_phase_based.at(ee)->ConvertPhaseToPolyDurations(
          phase_durations_.at(ee)->GetPhaseDurations());

      node_spline.push_back(
          std::make_shared<NodeSpline>(nodes_phase_based.at(ee).get(), ee_phase_poly_durations));
    }
  }

}

void SplineHolderExtended::InitializeJointMotion(std::vector<NodesVariables::Ptr> joint_motion,
                                                 const std::vector<double>& joint_poly_durations)
{

  for (int i = 0; i < joint_motion.size(); ++i)
    joint_motion_.push_back(
        std::make_shared<NodeSpline>(joint_motion.at(i).get() /* this get() is prolly a bad idea */,
                                     joint_poly_durations));  //base poly durations are the same as the one for the EE
}

void SplineHolderExtended::InitializeBaseAngMotion(NodesVariables::Ptr base_ang,
                                                   const std::vector<double>& base_poly_durations)
{
  base_angular_ = std::make_shared<NodeSpline>(base_ang.get(), base_poly_durations);
}

void SplineHolderExtended::InitializeBaseLinMotion(NodesVariables::Ptr base_lin,
                                                   const std::vector<double>& base_poly_durations)
{
  base_linear_ = std::make_shared<NodeSpline>(base_lin.get(), base_poly_durations);
}

void SplineHolderExtended::InitializeEEMotion(std::vector<NodesVariablesPhaseBased::Ptr> ee_motion_nodes, bool durations_change)
{
  InitializePhaseBasedDurations(ee_motion_nodes, durations_change, ee_motion_);
}

void SplineHolderExtended::InitializeEEForce(std::vector<NodesVariablesPhaseBased::Ptr> ee_force_nodes, bool durations_change)
{
  InitializePhaseBasedDurations(ee_force_nodes, durations_change, ee_force_);
}

void SplineHolderExtended::InitializePhaseDurations(
    std::vector<PhaseDurations::Ptr> phase_durations)
{
  phase_durations_ = phase_durations;\
  phase_durations_initialized_ = true;
}



} /* namespace */

