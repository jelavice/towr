/*
 * m545_model.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/models/examples/m545_model.h>

namespace towr {

constexpr unsigned int numEE = 5;

M545KinematicModelFull::M545KinematicModelFull(const std::string &urdfDescription, double dt)
    : KinematicModel(numEE),
      model_(dt)
{

  //get the excavator model
  model_.initModelFromUrdf(urdfDescription);

  //get the joint limits
  InitializeJointLimits();
  //PrintJointLimits();

  ee_pos_.resize(numEE);
  ee_jac_.resize(numEE);

//  std::cout << model_.getState() << std::endl;
//  Eigen::VectorXd jointAngles(static_cast<unsigned int>(NUM_JOINTS));
//  jointAngles.setZero();
//  GetEEPositions(jointAngles);

}

void M545KinematicModelFull::InitializeJointLimits()
{
  excavator_model::Limits limits;
  limits.init();

  // get the limits LF
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::LF, legDof);

  // get the limits RF
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::RF, legDof);

  // get the limits LH
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::LH, legDof);

  // get the limits RH
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::RH, legDof);

  // get the limits BOOM
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::BOOM, boomDof);  //a bit hacky

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
  GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::LF, loco_m545::RD::BodyEnum::LF_WHEEL,
                               jointAngles.segment(LimbStartIndex::LF, legDof), legDof);
  //RF
  GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::RF, loco_m545::RD::BodyEnum::RF_WHEEL,
                               jointAngles.segment(LimbStartIndex::RF, legDof), legDof);
  //LH
  GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::LH, loco_m545::RD::BodyEnum::LH_WHEEL,
                               jointAngles.segment(LimbStartIndex::LH, legDof), legDof);
  //RH
  GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::RH, loco_m545::RD::BodyEnum::RH_WHEEL,
                               jointAngles.segment(LimbStartIndex::RH, legDof), legDof);

  //BOOM
  GetEEPositionForSpecificLimb(loco_m545::RD::LimbEnum::BOOM, loco_m545::RD::BodyEnum::ENDEFFECTOR,
                               jointAngles.segment(LimbStartIndex::BOOM, boomDof), boomDof);

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
//jointPositions.block(idStart, 0, dof, 1) = jointAngles;
  jointPositions.segment(idStart, dof) = jointAngles;

//std::cout << "Joint positions: " << jointPositions.block(idStart,0,dof,1).transpose() << std::endl;
  state.getJointPositions().toImplementation() = jointPositions;
  model_.setState(state, true, false, false);
  Eigen::Vector3d positionEE = model_.getPositionBodyToBody(
      loco_m545::RD::BodyEnum::BASE, ee, loco_m545::RD::CoordinateFrameEnum::BASE);

  ee_pos_.at(static_cast<unsigned int>(limb)) = positionEE;

}

//todo method for jacobian calculation

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetEEJacobians(
    const VectorXd &jointAngles)
{
  return ee_jac_;
}

}
/*namespace*/
