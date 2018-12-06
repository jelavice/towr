/*
 * nodes_variables_ee_motion_with_wheels.cpp
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#include "towr/variables/nodes_variables_ee_motion_with_wheels.h"

namespace towr {

NodesVariablesEEMotionWithWheels::NodesVariablesEEMotionWithWheels(int n_nodes, int n_dim, std::string variable_id,
                                               int ee)
    : Base(n_nodes, n_dim, variable_id)
{
  ee_ = ee;
}

} /* namespace */



