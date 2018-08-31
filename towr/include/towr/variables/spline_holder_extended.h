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



  std::vector<NodeSpline::Ptr> joint_motion_; // for each ee there is one node spline class




};





} /* namespace */
