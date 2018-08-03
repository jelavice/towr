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

  //get the joint limits
  InitializeJointLimits();
  //PrintJointLimits();

  ee_pos_.resize(5);
  std::cout << model_.getState() << std::endl;

}

void M545KinematicModelFull::InitializeJointLimits()
{
  excavator_model::Limits limits;
  limits.init();

  // get the limits LF
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::LF, 3);

  // get the limits RF
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::RF, 3);

  // get the limits LH
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::LH, 3);

  // get the limits RH
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::RH, 3);

  // get the limits BOOM
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::BOOM, NUM_JOINTS - 12);  //a bit hacky

}

void M545KinematicModelFull::CalculateJointLimitsforSpecificLimb(
    const excavator_model::Limits &limits, loco_m545::RD::LimbEnum limb, unsigned int dof)
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
    upper_joint_limits_(id) = joint_limits_["upperLimit"][currJoint];
    lower_joint_limits_(id) = joint_limits_["lowerLimit"][currJoint];
    ++id;
  }
}

void M545KinematicModelFull::PrintJointLimits()
{

  for (auto it = joint_limits_.cbegin(); it != joint_limits_.cend(); ++it)
    for (auto jt = it->second.begin(); jt != it->second.cend(); ++jt)
      std::cout << it->first << " for joint " << jt->first << ": " << jt->second << std::endl;

  std::cout << "Lower limits joints vector: " << std::endl;
  std::cout << lower_joint_limits_ << std::endl << std::endl << std::endl;
  std::cout << "Upper limits joints vector: " << std::endl;
  std::cout << upper_joint_limits_ << std::endl;

}

const M545KinematicModelFull::JointVector &M545KinematicModelFull::GetLowerLimits()
{
  return lower_joint_limits_;
}
const M545KinematicModelFull::JointVector &M545KinematicModelFull::GetUpperLimits()
{
  return upper_joint_limits_;
}

const M545KinematicModelFull::EEPos &M545KinematicModelFull::GetEEPositions(
    const VectorXd &jointAngles)
{
  //get EE positions for LF
  {
    unsigned int dof = 3;
    GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::LF, loco_m545::RD::BodyEnum::LF_WHEEL,
                                 jointAngles.segment(LimbStartIndex::LF, dof), dof);
    //RF
    GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::RF, loco_m545::RD::BodyEnum::RF_WHEEL,
                                 jointAngles.segment(LimbStartIndex::RF, dof), dof);
    //LH
    GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::LH, loco_m545::RD::BodyEnum::LH_WHEEL,
                                 jointAngles.segment(LimbStartIndex::LH, dof), dof);
    //RH
    GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::RH, loco_m545::RD::BodyEnum::RH_WHEEL,
                                 jointAngles.segment(LimbStartIndex::RH, dof), dof);
  }

  {
    //BOOM
    unsigned int dof = 5;
    GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::BOOM,
                                 loco_m545::RD::BodyEnum::ENDEFFECTOR,
                                 jointAngles.segment(LimbStartIndex::BOOM, dof), dof);
  }

  return ee_pos_;

}

void M545KinematicModelFull::GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum limb,
                                                          loco_m545::RD::BodyEnum ee,
                                                          const Eigen::VectorXd &jointAngles,
                                                          unsigned int dof)
{
  excavator_model::JointVectorD jointPositions;
  excavator_model::ExcavatorState state = model_.getState();

  unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(limb));

  jointPositions = state.getJointPositions().toImplementation();
  jointPositions.block(idStart, 0, dof, 1) = jointAngles;
  //std::cout << "Joint positions: " << jointPositions.block(idStart,0,dof,1).transpose() << std::endl;
  state.getJointPositions().toImplementation() = jointPositions;
  model_.setState(state, true, false, false);
  Eigen::Vector3d positionEE = model_.getPositionBodyToBody(
      loco_m545::RD::BodyEnum::BASE, ee, loco_m545::RD::CoordinateFrameEnum::BASE);

  ee_pos_.at(static_cast<unsigned int>(limb)) = positionEE;

}

//todo method for jacobian calculation

}
/*namespace*/
