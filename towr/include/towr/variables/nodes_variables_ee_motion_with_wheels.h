/*
 * nodes_variables_ee_motion_with_wheels.h
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#pragma once


#include "towr/variables/nodes_variables_all.h"
#include "towr/variables/spline_holder.h"
namespace towr {

struct NonPhaseNodeInfo{
  bool isInContact = true;
};

class NodesVariablesEEMotionWithWheels : public NodesVariablesAll
{
 public:

  using Base = NodesVariablesAll;

  //todo add here the book keeping
  NodesVariablesEEMotionWithWheels(const SplineHolder &s, int n_nodes, int n_dim, std::string variable_id, int ee );

 private:

  int ee_;
  PhaseDurations::Ptr phase_durations_;
  std::vector<double> ee_polynomial_durations_;
  std::vector<NonPhaseNodeInfo> node_info_;


};

} /* namespace */
