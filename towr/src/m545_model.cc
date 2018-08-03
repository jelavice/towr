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

  UpdateModel(jointAngles);

  {
    //LF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
    ee_pos_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                     loco_m545::RD::BodyEnum::LF_WHEEL,
                                                     loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //RF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
    ee_pos_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                     loco_m545::RD::BodyEnum::RF_WHEEL,
                                                     loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //LH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
    ee_pos_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                     loco_m545::RD::BodyEnum::LH_WHEEL,
                                                     loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //RH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
    ee_pos_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                     loco_m545::RD::BodyEnum::RH_WHEEL,
                                                     loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //BOOM
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
    ee_pos_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                     loco_m545::RD::BodyEnum::ENDEFFECTOR,
                                                     loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  return ee_pos_;

}

//todo method for jacobian calculation

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetTranslationalJacobians(
    const VectorXd &jointAngles)
{

  UpdateModel(jointAngles);

  //todo extract only the entries I need!!!!
  MatrixXd tempJacobian;

  {
    //LF

    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
    model_.getJacobianTranslationFloatingBaseToBody(tempJacobian,
                                                    loco_m545::RD::BranchEnum::LF,
                                                    loco_m545::RD::BodyNodeEnum::WHEEL,
                                                    loco_m545::RD::CoordinateFrameEnum::BASE);

  }

  {
    //RF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
    model_.getJacobianTranslationFloatingBaseToBody(tempJacobian,
                                                    loco_m545::RD::BranchEnum::RF,
                                                    loco_m545::RD::BodyNodeEnum::WHEEL,
                                                    loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //LH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
    model_.getJacobianTranslationFloatingBaseToBody(tempJacobian,
                                                    loco_m545::RD::BranchEnum::LH,
                                                    loco_m545::RD::BodyNodeEnum::WHEEL,
                                                    loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //RH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
    model_.getJacobianTranslationFloatingBaseToBody(tempJacobian,
                                                    loco_m545::RD::BranchEnum::RH,
                                                    loco_m545::RD::BodyNodeEnum::WHEEL,
                                                    loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
      //BOOM
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian,
                                                      loco_m545::RD::BranchEnum::BOOM,
                                                      loco_m545::RD::BodyNodeEnum::ENDEFFECTOR,
                                                      loco_m545::RD::CoordinateFrameEnum::BASE);
    }

  return ee_jac_;
}



void M545KinematicModelFull::UpdateModel(const VectorXd &jointAngles)
{
  //LF
  UpdateSpecificLimb(loco_m545::RD::LimbEnum::LF, jointAngles.segment(LimbStartIndex::LF, legDof),
                     legDof);
  //RF
  UpdateSpecificLimb(loco_m545::RD::LimbEnum::RF, jointAngles.segment(LimbStartIndex::RF, legDof),
                     legDof);
  //LH
  UpdateSpecificLimb(loco_m545::RD::LimbEnum::LH, jointAngles.segment(LimbStartIndex::LH, legDof),
                     legDof);
  //RH
  UpdateSpecificLimb(loco_m545::RD::LimbEnum::RH, jointAngles.segment(LimbStartIndex::RH, legDof),
                     legDof);
  //BOOM
  UpdateSpecificLimb(loco_m545::RD::LimbEnum::BOOM,
                     jointAngles.segment(LimbStartIndex::BOOM, boomDof), boomDof);
}

void M545KinematicModelFull::UpdateSpecificLimb(loco_m545::RD::LimbEnum limb,
                                                const VectorXd &jointAngles, unsigned int dof)
{
  excavator_model::JointVectorD jointPositions;
  excavator_model::ExcavatorState state = model_.getState();

  unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(limb));
  jointPositions = state.getJointPositions().toImplementation();
//jointPositions.block(idStart, 0, dof, 1) = jointAngles;
  jointPositions.segment(idStart, dof) = jointAngles;
  state.getJointPositions().toImplementation() = jointPositions;
  model_.setState(state, true, false, false);
}

void M545KinematicModelFull::ExtractOptimizedJoints(const MatrixXd &bigJacobian){

//  {
//    unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(loco_m545::RD::LimbEnum::LF));
//    //copy LF
//  }

}

}
/*namespace*/
