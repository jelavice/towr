/*
 * m545_model.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/models/examples/m545_model.h>

namespace towr {

M545KinematicModelFull::M545KinematicModelFull(const std::string &urdfDescription, double dt)
    : KinematicModel(5),
      model_(dt)
{

  //get the excavator model
  model_.initModelFromUrdf(urdfDescription);

  //std::cout << urdfDescription << std::endl;

  //get the joint limits

  InitializeJointLimits();
  PrintJointLimits();

}

void M545KinematicModelFull::InitializeJointLimits()
{
  excavator_model::Limits limits;
  limits.init();


  // get the limits LF
  CalculateJointLimitsforSpecificLeg(limits, loco_m545::RD::LimbEnum::LF, 3);

  // get the limits RF
  CalculateJointLimitsforSpecificLeg(limits, loco_m545::RD::LimbEnum::RF, 3);

  // get the limits LH
  CalculateJointLimitsforSpecificLeg(limits, loco_m545::RD::LimbEnum::LH, 3);

  // get the limits RH
  CalculateJointLimitsforSpecificLeg(limits, loco_m545::RD::LimbEnum::RH, 3);

  // get the limits BOOM
  CalculateJointLimitsforSpecificLeg(limits, loco_m545::RD::LimbEnum::BOOM, NUM_JOINTS - 12); //a bit hacky


}

void M545KinematicModelFull::CalculateJointLimitsforSpecificLeg(const excavator_model::Limits &limits,
                                                          loco_m545::RD::LimbEnum limb,
                                                          unsigned int dof)
{

  static unsigned int id = 0;

  unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(limb));
  unsigned int idEnd = idStart + dof;

  for (unsigned int jointId = idStart; jointId < idEnd; ++jointId) {

    loco_m545::RD::JointEnum jointEnum = static_cast<loco_m545::RD::JointEnum>(jointId);
    double upperLimit = limits.getJointMaxPosition(jointEnum);
    double lowerLimit = limits.getJointMinPosition(jointEnum);
    std::string currJoint = loco_m545::RD::mapKeyEnumToKeyName(jointEnum);
    joint_limits_["lowerLimit"][currJoint] = std::min<double>(lowerLimit, upperLimit);
    joint_limits_["upperLimit"][currJoint] = std::max<double>(lowerLimit, upperLimit);
    upperJointLimits_(id) = joint_limits_["upperLimit"][currJoint];
    lowerJointLimits_(id) = joint_limits_["lowerLimit"][currJoint];
    ++id;
  }
}

void M545KinematicModelFull::PrintJointLimits(){

  for (auto it = joint_limits_.cbegin(); it != joint_limits_.cend(); ++it)
    for (auto jt = it->second.begin(); jt != it->second.cend(); ++jt)
        std::cout << it->first << " for joint " << jt->first << ": " << jt->second << std::endl;

  std::cout << "Lower limits joints vector: " << std::endl;
  std::cout << lowerJointLimits_ << std::endl << std::endl << std::endl;
  std::cout << "Upper limits joints vector: " << std::endl;
  std::cout << upperJointLimits_ << std::endl;

}




//todo method for getting the joint limits

//todo method for EE pos

//todo method for jacobian calculation

}
/*namespace*/
