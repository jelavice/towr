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

#include "excavator_model/ExcavatorModel.hpp"
#include "excavator_model/Limits.hpp"

namespace towr {

/**
 * \addtogroup Robots
 * @{
 */
class M545KinematicModelFull : public KinematicModel {
public:

  using MatrixXd = Eigen::MatrixXd;

  M545KinematicModelFull (const std::string &urdfDescription, double dt);
  EEPos GetEEPositions(VectorXd jointAngles);
  MatrixXd GetEEJacobian(VectorXd jointAngles);


private:
  excavator_model::Limits joint_limits_;
  excavator_model::ExcavatorModel model_;

};

class M545DynamicModelFull : public SingleRigidBodyDynamics {
public:
  M545DynamicModelFull() : SingleRigidBodyDynamics(3700.0,
                      2485.29, 1640.58, 1600.99, 0.0, 0.0, 0.0,
                      4) {}
};
/** @}*/

} /* namespace towr */
