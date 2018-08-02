/*
 * m545_model.h
 *
 *  Created on: Aug 2, 2018
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
class M545KinematicModelFull : public KinematicModel {
public:
  M545KinematicModelFull () : KinematicModel(4)
  {

    //todo implement so much crap here

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
