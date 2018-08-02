/*
 * m545_model.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/models/examples/m545_model.h>

namespace towr {

M545KinematicModelFull::M545KinematicModelFull(const std::string &urdfDescription, double dt)
    : KinematicModel(5), model_(dt)
{
  //todo initialize model from urdf

  //todo add all the joint angles limits

  //get the excavator model

    model_.initModelFromUrdf(urdfDescription);

  //get the joint limits
    excavator_model::Limits limits;
    limits.init();

}


//todo method for getting the joint limits

//todo method for EE pos

//todo method for jacobian calculation





}
/*namespace*/
