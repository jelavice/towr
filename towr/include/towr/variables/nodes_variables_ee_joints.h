/*
 * NodesVaraiblesEEJoints.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/variables/nodes_variables_all.h"

namespace towr {

class NodesVariablesEEJoints : public NodesVariablesAll
{
 public:

  using Base = NodesVariablesAll;

  NodesVariablesEEJoints(int n_nodes, int n_dim, std::string variable_id, int ee);

 private:

  int ee_;

};

} /* namespace */
