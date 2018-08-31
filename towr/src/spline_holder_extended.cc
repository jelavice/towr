/*
 * spline_holder_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/variables/spline_holder_extended.h"

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

void SplineHolderExtended::InitializeJointMotion(std::vector<NodesVariables::Ptr> joint_motion, const std::vector<double>& joint_poly_durations){

  for (int i = 0; i < joint_motion.size(); ++i)
      joint_motion_.push_back(
          std::make_shared<NodeSpline>(joint_motion.at(i).get(), joint_poly_durations)); //base poly durations are the same as the one for the EE
}

}
/* namespace */

