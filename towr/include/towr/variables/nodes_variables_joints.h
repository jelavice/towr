/*
 * nodes_variables_joints.h
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */

#pragma once


#include "nodes_variables.h"

namespace towr {


class NodesVariablesJoint : public NodesVariables {
public:
  /**
   * @param n_nodes  Number of nodes to construct the spline.
   * @param n_dim    Number of dimensions of each node.
   * @param variable_id  Name of this variables set in the optimization.
   */
  NodesVariablesJoint (int n_nodes, int n_dim, std::string variable_id, int ee);
  virtual ~NodesVariablesJoint () = default;

  std::vector<NodeValueInfo> GetNodeValuesInfo(int idx) const override;

private:
  int ee_;


};

} /* namespace towr */
