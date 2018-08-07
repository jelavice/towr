/*
 * nodes_variables_joint.cc
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */


#include <towr/variables/nodes_variables_joints.h>

namespace towr {

//todo implement

NodesVariablesAll::NodesVariablesAll (int n_nodes, int n_dim, std::string variable_id)
    : NodesVariables(variable_id)
{
  int n_opt_variables = n_nodes*Node::n_derivatives*n_dim;

  n_dim_ = n_dim;
  nodes_  = std::vector<Node>(n_nodes, Node(n_dim));
  bounds_ = VecBound(n_opt_variables, ifopt::NoBound);
  SetRows(n_opt_variables);
}

std::vector<NodesVariablesAll::NodeValueInfo>
NodesVariablesAll::GetNodeValuesInfo (int idx) const
{
  std::vector<NodeValueInfo> vec_nvi;

  int n_opt_values_per_node_ = 2*GetDim();
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  NodeValueInfo nvi;
  nvi.deriv_ = internal_id<GetDim()? kPos : kVel;
  nvi.dim_   = internal_id%GetDim();
  nvi.id_    = std::floor(idx/n_opt_values_per_node_);

  vec_nvi.push_back(nvi);

  return vec_nvi;
}

} /* namespace */
