/*
 * nodes_variables_joint.cc
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */


#include "towr/variables/nodes_variables_limb_joints.h"

namespace towr {


NodesVariablesLimbJoints::NodesVariablesLimbJoints (int n_nodes, int n_dim, std::string variable_id, int ee)
    : NodesVariablesAll(n_nodes, n_dim, variable_id)
{
  int n_opt_variables = n_nodes*Node::n_derivatives*n_dim;

  ee_ = ee;

}



} /* namespace */
