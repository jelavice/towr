/*
 * nodes_variables_ee_joints.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/variables/nodes_variables_ee_joints.h"

namespace towr {

NodesVariablesEEJoints::NodesVariablesEEJoints(int n_nodes, int n_dim, std::string variable_id,
                                               int ee)
    : Base(n_nodes, n_dim, variable_id)
{
  ee_ = ee;
}

} /* namespace */

