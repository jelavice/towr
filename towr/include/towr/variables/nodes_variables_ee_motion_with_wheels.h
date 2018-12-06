/*
 * nodes_variables_ee_motion_with_wheels.h
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#pragma once


#include "towr/variables/nodes_variables_all.h"

namespace towr {

class NodesVariablesEEMotionWithWheels : public NodesVariablesAll
{
 public:

  using Base = NodesVariablesAll;

  NodesVariablesEEMotionWithWheels(int n_nodes, int n_dim, std::string variable_id, int ee);

 private:

  int ee_;

};

} /* namespace */
