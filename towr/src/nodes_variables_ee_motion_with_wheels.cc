/*
 * nodes_variables_ee_motion_with_wheels.cpp
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#include "towr/variables/nodes_variables_ee_motion_with_wheels.h"

namespace towr {

NodesVariablesEEMotionWithWheels::NodesVariablesEEMotionWithWheels(const SplineHolder &s, int n_nodes, int n_dim, std::string variable_id,
                                               int ee)
    : Base(n_nodes, n_dim, variable_id)
{
  ee_ = ee;
  phase_durations_ = s.phase_durations_.at(ee);
  //todo need to compute the end effector polynomial durations
  // I can do that prolly similar as it is done for the base polynomials
  // prolly I can directly use the polynomials that I calculated already for the joint variables

  // then look through the phase durations and then mark the nodes

  //todo fill this guy node_info_

}

} /* namespace */



