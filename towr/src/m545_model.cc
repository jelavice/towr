/*
 * m545_model.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/models/examples/m545_model.h>

#include <kindr/Core>

//#define M545MODELDEBUG

namespace towr {

M545KinematicModelFull::M545KinematicModelFull(const std::string &urdfDescription, double dt)
    : KinematicModelJoints(numEE),
      model_(dt)
{

  //allocate memory and shit
  upper_joint_limits_.resize(NUM_JOINTS);
  lower_joint_limits_.resize(NUM_JOINTS);

  //get the excavator model
  model_.initModelFromUrdf(urdfDescription);

  //get the joint limits
  CalculateJointLimits();
  //PrintJointLimits();

  //resize arrays
  ee_pos_base_.resize(numEE);

  /*bas coordinate frame */
  ee_trans_jac_joints_base_.resize(numEE);

  //create sparse matrices
  for (int i = 0; i < numEE; ++i) {

    int numDof = num_dof_limbs_.at(i);

    /* jacobian in base frame */
    ee_trans_jac_joints_base_.at(i).resize(3, numDof);
  }

  // initialize with zero

  //todo udpate model
  for (int i = 0; i < numEE; ++i) {
    Eigen::VectorXd jointAngles(num_dof_limbs_.at(i));
    jointAngles.setZero();
    UpdateModel(jointAngles, i);
  }

  max_dev_from_nominal_.setZero();

  nominal_stance_ = ee_pos_base_;

#ifdef M545MODELDEBUG

  std::cout << model_.getState() << std::endl << std::endl;

  std::cout << "feet positions in base: " << std::endl;
  for (int i = 0; i < numEE; ++i) {
    std::cout << GetEEPositionsBase(i).transpose() << std::endl << std::endl;
  }

  std::cout << "Translational jacobians wrt joints base: " << std::endl;
  for (int i = 0; i < numEE; ++i) {
    std::cout << ee_trans_jac_joints_base_.at(i) << std::endl << std::endl;
  }

#endif

}

void M545KinematicModelFull::CalculateJointLimits()
{
  excavator_model::Limits limits;
  limits.init();

  // get the limits LF
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::LF, legDof);

  // get the limits RF
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::RF, legDof);

  // get the limits LH
  CalculateJointLimitsforSpecificLimb(limits, loco_m545::RD::LimbEnum::LH, legDof);

  // get the limits RHM545MODELDEBUG
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

int M545KinematicModelFull::getLimbStartingId(int LimbId)
{
  switch (LimbId) {

    case (0):
      return static_cast<int>(LimbStartIndex::LF);
    case (1):
      return static_cast<int>(LimbStartIndex::RF);
    case (2):
      return static_cast<int>(LimbStartIndex::LH);
    case (3):
      return static_cast<int>(LimbStartIndex::RH);
    case (4):
      return static_cast<int>(LimbStartIndex::BOOM);
    default:
      return -1;

  }
}

M545KinematicModelFull::VectorXd M545KinematicModelFull::GetLowerJointLimits(int limbId)
{
  return lower_joint_limits_.segment(getLimbStartingId(limbId), num_dof_limbs_.at(limbId));
}

M545KinematicModelFull::VectorXd M545KinematicModelFull::GetUpperJointLimits(int limbId)
{
  return upper_joint_limits_.segment(getLimbStartingId(limbId), num_dof_limbs_.at(limbId));
}



Eigen::Vector3d M545KinematicModelFull::GetEEPositionsBase(int limbId)
{

  switch (limbId) {
    case 0: {
      //LF
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
      ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(
          loco_m545::RD::BodyEnum::BASE, loco_m545::RD::BodyEnum::LF_WHEEL,
          loco_m545::RD::CoordinateFrameEnum::BASE);
      break;
    }

    case 1: {
      //RF
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
      ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(
          loco_m545::RD::BodyEnum::BASE, loco_m545::RD::BodyEnum::RF_WHEEL,
          loco_m545::RD::CoordinateFrameEnum::BASE);
      break;
    }

    case 2: {
      //LH
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
      ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(
          loco_m545::RD::BodyEnum::BASE, loco_m545::RD::BodyEnum::LH_WHEEL,
          loco_m545::RD::CoordinateFrameEnum::BASE);
      break;
    }

    case 3: {
      //RH
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
      ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(
          loco_m545::RD::BodyEnum::BASE, loco_m545::RD::BodyEnum::RH_WHEEL,
          loco_m545::RD::CoordinateFrameEnum::BASE);
      break;
    }

    case 4: {
      //BOOM
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
      ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(
          loco_m545::RD::BodyEnum::BASE, loco_m545::RD::BodyEnum::ENDEFFECTOR,
          loco_m545::RD::CoordinateFrameEnum::BASE);
      break;
    }

  }


  // subtract the radius of the wheel to get the contact point
  // this assumes of course that the ground is always in a plane
  // spanned by x and y axis, below the base frame
  for (int i = 0; i < 4; ++i)
    ee_pos_base_.at(i).z() -= model_.getWheelRadius();

  return ee_pos_base_.at(limbId);

}

void M545KinematicModelFull::CalculateTranslationalJacobiansWRTjointsBase(int limbId)
{

  loco_m545::RD::CoordinateFrameEnum coordinate_system = loco_m545::RD::CoordinateFrameEnum::BASE;

  switch (limbId) {

    case 0: {
      //LF
      MatrixXd tempJacobian(3, model_.getDofCount());
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::LF,
                                                      loco_m545::RD::BodyNodeEnum::WHEEL,
                                                      coordinate_system);

      //      std::cout << "Dof count: " << model_.getDofCount() << std::endl;
      //      std::cout << "Jacobian LF \n" << tempJacobian.transpose() << std::endl << std::endl;
      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LF, LimbStartIndex::LF,
                                  legDof, ee_trans_jac_joints_base_);
      break;
    }
    case 1: {
      //RF
      MatrixXd tempJacobian(3, model_.getDofCount());
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::RF,
                                                      loco_m545::RD::BodyNodeEnum::WHEEL,
                                                      coordinate_system);

      //      std::cout << "Jacobian RF \n" << tempJacobian.transpose() << std::endl;
      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RF, LimbStartIndex::RF,
                                  legDof, ee_trans_jac_joints_base_);
      break;
    }

    case 2: {
      //LH
      MatrixXd tempJacobian(3, model_.getDofCount());
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::LH,
                                                      loco_m545::RD::BodyNodeEnum::WHEEL,
                                                      coordinate_system);

      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LH, LimbStartIndex::LH,
                                  legDof, ee_trans_jac_joints_base_);
      break;
    }

    case 3: {
      //RH
      MatrixXd tempJacobian(3, model_.getDofCount());
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::RH,
                                                      loco_m545::RD::BodyNodeEnum::WHEEL,
                                                      coordinate_system);

      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RH, LimbStartIndex::RH,
                                  legDof, ee_trans_jac_joints_base_);
      break;
    }

    case 4: {
      //BOOM
      MatrixXd tempJacobian(3, model_.getDofCount());
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::BOOM,
                                                      loco_m545::RD::BodyNodeEnum::ENDEFFECTOR,
                                                      coordinate_system);

      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::BOOM, LimbStartIndex::BOOM,
                                  boomDof, ee_trans_jac_joints_base_);
      break;
    }
  }

}

void M545KinematicModelFull::UpdateModel(VectorXd jointAngles, int limbId)
{
  switch (limbId) {
    case 0: {
      //LF
      UpdateSpecificLimb(loco_m545::RD::LimbEnum::LF, jointAngles, legDof);
      break;
    }

    case 1: {
      //RF
      UpdateSpecificLimb(loco_m545::RD::LimbEnum::RF, jointAngles, legDof);
      break;
    }

    case 2: {
      //LH
      UpdateSpecificLimb(loco_m545::RD::LimbEnum::LH, jointAngles, legDof);
      break;
    }

    case 3: {
      //RH
      UpdateSpecificLimb(loco_m545::RD::LimbEnum::RH, jointAngles, legDof);
      break;
    }

    case 4: {
      //BOOM
      UpdateSpecificLimb(loco_m545::RD::LimbEnum::BOOM, jointAngles, boomDof);
      break;
    }

  }

  CalculateTranslationalJacobiansWRTjointsBase(limbId);
}

void M545KinematicModelFull::UpdateSpecificLimb(loco_m545::RD::LimbEnum limb,const VectorXd &jointAngles,
                                                unsigned int dof)
{
  excavator_model::JointVectorD jointPositions;
  excavator_model::ExcavatorState state = model_.getState();

  unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(limb));

  jointPositions = state.getJointPositions().toImplementation();
  jointPositions.segment(idStart, dof) = jointAngles;
  state.getJointPositions().toImplementation() = jointPositions;
  //todo this line was woring, it wouldn't update the joints, see whether that is a bug in loco/kindr
  //state.getJointPositions().toImplementation().segment(idStart, dof) = jointPositions
  model_.setState(state, true, false, false);
}

void M545KinematicModelFull::ExtractJointJacobianEntries(const MatrixXd &bigJacobian,
                                                         loco_m545::RD::LimbEnum limb,
                                                         LimbStartIndex limbStartIndex,
                                                         unsigned int dof, EEJac &jacArray)
{

  constexpr unsigned int dim3 = 3;
  constexpr unsigned int offset = 6;  // ofset for the first columns of the jacobian (base linear + angular)

  unsigned int ee_id = static_cast<unsigned int>(limb);
  unsigned int idStart = static_cast<unsigned int>(limbStartIndex);
  unsigned int idStart_loco = loco_m545::RD::mapKeyEnumToKeyId(
      loco_m545::RD::getLimbStartJoint(limb)) + offset;

//  std::cout << "limb id: " << ee_id << std::endl;
//  std::cout << "id start: " << idStart << std::endl;
//  std::cout << "id start loco: " << idStart_loco << std::endl;
//  std::cout << "Jacobian size: " << bigJacobian.rows() << " x " << bigJacobian.cols() << std::endl;
//  std::cout << "sparse matrix size " << ee_trans_jac_joints_.at(ee_id).rows() << " x " << ee_trans_jac_joints_.at(ee_id).cols() << std::endl;
  // std::cout << "bi jacobian \n: " << bigJacobian.transpose() << std::endl;

  for (unsigned int j = 0; j < dim3; ++j)
    for (unsigned int i = 0; i < dof; ++i) {
      //std::cout <<  "coeffs: " << idStart + i << ", " << idStart_loco + i << std::endl;
//      jacArray.at(ee_id).coeffRef(j, idStart + i) = bigJacobian(j, idStart_loco + i);
      jacArray.at(ee_id).coeffRef(j, i) = bigJacobian(j, idStart_loco + i);

      //std::cout << "in the big matrix: " << bigJacobian(j, idStart_loco + i) << std::endl;
      //std::cout << "In the sparse matrix: " << jacArray.at(ee_id).coeffRef(j, idStart + i) << std::endl;
      //std:: cout << " done" << std::endl;
    }
  //std::cout << std::endl << std::endl;
  //std::cout << "after copying my stuff: " << ee_trans_jac_joints_base_.at(ee_id) << std::endl;

}




M545KinematicModelFull::SparseMatrix M545KinematicModelFull::GetTranslationalJacobiansWRTjointsBase(
    int limbId)
{
  return ee_trans_jac_joints_base_.at(limbId);
}

void M545KinematicModelFull::printCurrentJointPositions(){

  std::cout << "Joint positions: " << model_.getState().getJointPositions().toImplementation().transpose() << std::endl;

}


}
/*namespace*/
