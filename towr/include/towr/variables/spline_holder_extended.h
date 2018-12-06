/*
 * spline_holder_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/variables/spline_holder.h"


namespace towr {

struct SplineHolderExtended : public SplineHolder {



  using Base = SplineHolder;

  SplineHolderExtended() = default;


  SplineHolderExtended(NodesVariables::Ptr base_lin,
                       NodesVariables::Ptr base_ang,
                       const std::vector<double>& base_poly_durations,
                       std::vector<NodesVariablesPhaseBased::Ptr> ee_motion,
                       std::vector<NodesVariablesPhaseBased::Ptr> ee_force,
                       std::vector<PhaseDurations::Ptr> phase_durations,
                       bool ee_durations_change);


  void InitializeJointMotion(std::vector<NodesVariables::Ptr> joint_motion,  const std::vector<double>& joint_poly_durations);

  void InitializeBaseLinMotion(NodesVariables::Ptr base_lin, const std::vector<double>& base_poly_durations);

  void InitializeBaseAngMotion(NodesVariables::Ptr base_ang, const std::vector<double>& base_poly_durations);

  void InitializeEEMotion(std::vector<NodesVariablesPhaseBased::Ptr> ee_motion, bool durations_change);

  void InitializeEEForce(std::vector<NodesVariablesPhaseBased::Ptr> ee_force, bool durations_change);

  void InitializePhaseDurations(std::vector<PhaseDurations::Ptr> phase_durations);

  void InitializeEEMotionWithWheels(std::vector<NodesVariables::Ptr> ee_motion_with_wheels,
                                    const std::vector<double>& ee_motion_poly_durations);




  std::vector<NodeSpline::Ptr> joint_motion_; // for each ee there is one node spline class
  std::vector<NodeSpline::Ptr> ee_with_wheels_motion_; // for each ee there is one node spline class


 private:
  void InitializePhaseBasedDurations(std::vector<NodesVariablesPhaseBased::Ptr> nodes_phase_based,
                                     bool durations_change, std::vector<NodeSpline::Ptr> &node_spline);

  bool phase_durations_initialized_ = false;


};





} /* namespace */
