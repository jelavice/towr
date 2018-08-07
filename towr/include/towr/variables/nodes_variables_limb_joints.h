/*
 * nodes_variables_joints.h
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */

#pragma once


#include "nodes_variables_all.h"

namespace towr {


class NodesVariablesLimbJoints : public NodesVariablesAll {
public:

  NodesVariablesLimbJoints (int n_nodes, int n_dim, std::string variable_id, int ee);
  ~NodesVariablesLimbJoints () = default;

private:
  int ee_;


};

} /* namespace towr */
