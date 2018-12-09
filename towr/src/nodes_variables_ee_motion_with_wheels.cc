/*
 * nodes_variables_ee_motion_with_wheels.cpp
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#include "towr/variables/nodes_variables_ee_motion_with_wheels.h"

namespace towr {

NodesVariablesEEMotionWithWheels::NodesVariablesEEMotionWithWheels(const SplineHolder &s, const VecTimes &poly_duratinos, int n_nodes,
                                                                   int n_dim, const std::string &variable_id, int ee)
    : Base(n_nodes, n_dim, variable_id)
{
  ee_ = ee;
  phase_durations_ = s.phase_durations_.at(ee);
  n_nodes_ = n_nodes;

  ee_polynomial_durations_ = poly_duratinos;

  CalculateContactSchedule(n_nodes);

}


void NodesVariablesEEMotionWithWheels::CalculateContactSchedule(int n_nodes) {


  nodes_info_.reserve(n_nodes);

  //push the first node
  double global_time = 0;
  nodes_info_.push_back(NonPhaseNodeInfo(phase_durations_->IsContactPhase(global_time), global_time));


  for (int i=0; i < ee_polynomial_durations_.size(); ++i){
    global_time += ee_polynomial_durations_.at(i);
    nodes_info_.push_back(NonPhaseNodeInfo(phase_durations_->IsContactPhase(global_time), global_time));

  }

  assert(n_nodes == nodes_info_.size() && "n_nodes not equal to size of the vector");

}


} /* namespace */



