/*
 * m545_model_simple.h
 *
 *  Created on: Jul 17, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * \addtogroup Robots
 * @{
 */
class M545KinematicModel : public KinematicModel {
public:
  M545KinematicModel () : KinematicModel(4)
  {


    const double x_nominal_b_front = 2.7;
    const double y_nominal_b_front = 1.6;
    const double z_nominal_b = -0.95;

    nominal_stance_.at(LF) <<  x_nominal_b_front,   y_nominal_b_front, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b_front,  -y_nominal_b_front, z_nominal_b;

    const double x_nominal_b_hind = 2.05;
    const double y_nominal_b_hind = 1.26;

    nominal_stance_.at(LH) << -x_nominal_b_hind,   y_nominal_b_hind, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b_hind,  -y_nominal_b_hind, z_nominal_b;

    max_dev_from_nominal_ << 0.05, 0.05, 0.05;
  }
};

class M545DynamicModel : public SingleRigidBodyDynamics {
public:
  M545DynamicModel() : SingleRigidBodyDynamics(3700.0,
                      2485.29, 1640.58, 1600.99, 0.0, 0.0, 0.0,
                      4) {}
};
/** @}*/

} /* namespace towr */


