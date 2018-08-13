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
  ee_pos_.resize(numEE);
  ee_pos_base_.resize(numEE);
  ee_rot_.resize(numEE);

  //translatinal jacobians
  ee_trans_jac_joints_.resize(numEE);
  ee_trans_jac_base_orientation_.resize(numEE);
  ee_trans_jac_base_position_.resize(numEE);

  //rotational jacobians
  ee_rot_jac_joints_.resize(numEE);
  ee_rot_jac_base_orientation_.resize(numEE);

  /*bas coordinate frame */
  ee_trans_jac_joints_base_.resize(numEE);

  //create sparse matrices
  for (int i = 0; i < numEE; ++i) {

    int numDof = num_dof_limbs_.at(i);

    //rotational jacobians
    ee_rot_jac_joints_.at(i).resize(3, numDof);
    ee_rot_jac_base_orientation_.at(i).resize(3, 3);

    //translational jabional
    ee_trans_jac_joints_.at(i).resize(3, numDof);
    ee_trans_jac_base_orientation_.at(i).resize(3, 3);
    ee_trans_jac_base_position_.at(i).resize(3, 3);

    //rotational jacobians wrt to base angles are idenitiy
    ee_rot_jac_base_orientation_.at(i).setIdentity();
    ee_trans_jac_base_position_.at(i).setIdentity();

    /* jacobian in base frame */
    ee_trans_jac_joints_base_.at(i).resize(3,numDof);
  }

  //std::cout << model_.getState() << std::endl;

  // initialize with zero
  Eigen::VectorXd jointAngles(static_cast<unsigned int>(NUM_JOINTS));
  Eigen::Vector3d euler(3);
  Eigen::Vector3d position(3);
  jointAngles.setZero();
  euler.setZero();
  position.setZero();
  position.z() = 0.95;
  UpdateModel(jointAngles, euler, position);

  max_dev_from_nominal_.setZero();

#ifdef M545MODELDEBUG
  std::cout << "Translational jacobians wrt joints: " << std::endl;
  for (const auto &x : ee_trans_jac_joints_) {
    std::cout << x << std::endl << std::endl;
  }

  std::cout << "Translational jacobians wrt position: " << std::endl;
  for (const auto &x : ee_trans_jac_base_position_) {
    std::cout << x << std::endl << std::endl;
  }

  std::cout << "Translational jacobians wrt orientation: " << std::endl;
  for (const auto &x : ee_trans_jac_base_orientation_) {
    std::cout << x << std::endl << std::endl;
  }

  std::cout << "rotational jacobians wrt joints: " << std::endl;
  for (const auto &x : ee_rot_jac_joints_) {
    std::cout << x << std::endl << std::endl;
  }

  std::cout << "rotational jacobians wrt base angles: " << std::endl;
  for (const auto &x : ee_rot_jac_base_orientation_) {
    std::cout << x << std::endl << std::endl;
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

const M545KinematicModelFull::VectorXd M545KinematicModelFull::GetLowerJointLimits(int limbId)
{
  return lower_joint_limits_.segment(getLimbStartingId(limbId), num_dof_limbs_.at(limbId));
}
const M545KinematicModelFull::VectorXd M545KinematicModelFull::GetUpperJointLimits(int limbId)
{
  return upper_joint_limits_.segment(getLimbStartingId(limbId), num_dof_limbs_.at(limbId));
}

Eigen::Vector3d rotMat2ypr(const Eigen::Matrix3d &mat)
{

  // rotation convention for this is yaw pitch roll in that order

  kindr::RotationMatrixD rotMat(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2),
                                mat(2, 0), mat(2, 1), mat(2, 2));

  kindr::EulerAnglesYprD euler(rotMat);

  euler.setUnique();

  return Eigen::Vector3d(euler.x(), euler.y(), euler.z());

}

const M545KinematicModelFull::EEPos &M545KinematicModelFull::GetEEOrientation()
{

  {
    //LF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
    ee_rot_.at(ee_id) = rotMat2ypr(
        model_.getOrientationWorldToBody(loco_m545::RD::BodyEnum::LF_WHEEL));
  }

  {
    //RF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
    ee_rot_.at(ee_id) = rotMat2ypr(
        model_.getOrientationWorldToBody(loco_m545::RD::BodyEnum::RF_WHEEL));
  }

  {
    //LH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
    ee_rot_.at(ee_id) = rotMat2ypr(
        model_.getOrientationWorldToBody(loco_m545::RD::BodyEnum::LH_WHEEL));
  }

  {
    //RH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
    ee_rot_.at(ee_id) = rotMat2ypr(
        model_.getOrientationWorldToBody(loco_m545::RD::BodyEnum::RH_WHEEL));
  }

  {
    //LH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
    ee_rot_.at(ee_id) = rotMat2ypr(model_.getOrientationWorldToBody(loco_m545::RD::BodyEnum::BOOM));
  }

  return ee_rot_;

}

const M545KinematicModelFull::EEPos &M545KinematicModelFull::GetEEPositionsWorld()
{

  {
    //LF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
    ee_pos_.at(ee_id) = model_.getPositionWorldToBody(loco_m545::RD::BodyEnum::LF_WHEEL,
                                                      loco_m545::RD::CoordinateFrameEnum::WORLD);
  }

  {
    //RF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
    ee_pos_.at(ee_id) = model_.getPositionWorldToBody(loco_m545::RD::BodyEnum::RF_WHEEL,
                                                      loco_m545::RD::CoordinateFrameEnum::WORLD);
  }

  {
    //LH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
    ee_pos_.at(ee_id) = model_.getPositionWorldToBody(loco_m545::RD::BodyEnum::LH_WHEEL,
                                                      loco_m545::RD::CoordinateFrameEnum::WORLD);
  }

  {
    //RH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
    ee_pos_.at(ee_id) = model_.getPositionWorldToBody(loco_m545::RD::BodyEnum::RH_WHEEL,
                                                      loco_m545::RD::CoordinateFrameEnum::WORLD);
  }

  {
    //BOOM
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
    ee_pos_.at(ee_id) = model_.getPositionWorldToBody(loco_m545::RD::BodyEnum::ENDEFFECTOR,
                                                      loco_m545::RD::CoordinateFrameEnum::WORLD);
  }

  return ee_pos_;

}

const M545KinematicModelFull::EEPos &M545KinematicModelFull::GetEEPositionsBase()
{

  {
    //LF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
    ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                          loco_m545::RD::BodyEnum::LF_WHEEL,
                                                          loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //RF
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
    ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                          loco_m545::RD::BodyEnum::RF_WHEEL,
                                                          loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //LH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
    ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                          loco_m545::RD::BodyEnum::LH_WHEEL,
                                                          loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //RH
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
    ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                          loco_m545::RD::BodyEnum::RH_WHEEL,
                                                          loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  {
    //BOOM
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
    ee_pos_base_.at(ee_id) = model_.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                          loco_m545::RD::BodyEnum::ENDEFFECTOR,
                                                          loco_m545::RD::CoordinateFrameEnum::BASE);
  }

  for (int i=0; i < 4; ++i)
    ee_pos_base_.at(i).z() = -base_xyz_.z();


  return ee_pos_base_;

}

void M545KinematicModelFull::CalculateTranslationalJacobiansWRTjointsAndBaseOrientation()
{

  MatrixXd tempJacobian(3, model_.getDofCount());

  {
    //LF
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
    model_.getJacobianTranslationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::LF,
                                             loco_m545::RD::BodyNodeEnum::WHEEL,
                                             loco_m545::RD::CoordinateFrameEnum::WORLD);

//    std::cout << "Dof count: " << model_.getDofCount() << std::endl;
//    std::cout << "Jacobian LF \n" << tempJacobian.transpose() << std::endl << std::endl;
    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LF, LimbStartIndex::LF,
                                legDof, ee_trans_jac_joints_);
    ExtractOrientationJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LF,
                                      ee_trans_jac_base_orientation_);
  }

  {
    //RF
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
    model_.getJacobianTranslationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::RF,
                                             loco_m545::RD::BodyNodeEnum::WHEEL,
                                             loco_m545::RD::CoordinateFrameEnum::WORLD);

    //std::cout << "Jacobian RF \n" << tempJacobian.transpose() << std::endl;
    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RF, LimbStartIndex::RF,
                                legDof, ee_trans_jac_joints_);
    ExtractOrientationJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RF,
                                      ee_trans_jac_base_orientation_);

  }

  {
    //LH
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
    model_.getJacobianTranslationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::LH,
                                             loco_m545::RD::BodyNodeEnum::WHEEL,
                                             loco_m545::RD::CoordinateFrameEnum::WORLD);

    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LH, LimbStartIndex::LH,
                                legDof, ee_trans_jac_joints_);
    ExtractOrientationJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LH,
                                      ee_trans_jac_base_orientation_);

  }

  {
    //RH
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
    model_.getJacobianTranslationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::RH,
                                             loco_m545::RD::BodyNodeEnum::WHEEL,
                                             loco_m545::RD::CoordinateFrameEnum::WORLD);

    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RH, LimbStartIndex::RH,
                                legDof, ee_trans_jac_joints_);
    ExtractOrientationJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RH,
                                      ee_trans_jac_base_orientation_);

  }

  {
    //BOOM
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
    model_.getJacobianTranslationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::BOOM,
                                             loco_m545::RD::BodyNodeEnum::ENDEFFECTOR,
                                             loco_m545::RD::CoordinateFrameEnum::WORLD);

    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::BOOM, LimbStartIndex::BOOM,
                                boomDof, ee_trans_jac_joints_);
    ExtractOrientationJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::BOOM,
                                      ee_trans_jac_base_orientation_);

  }

}

void M545KinematicModelFull::CalculateOrientationJacobiansWRTjoints()
{

  MatrixXd tempJacobian(3, model_.getDofCount());

  {
    //LF
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);

    model_.getJacobianRotationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::LF,
                                          loco_m545::RD::BodyNodeEnum::WHEEL,
                                          loco_m545::RD::CoordinateFrameEnum::WORLD);

//    std::cout << "Dof count: " << model_.getDofCount() << std::endl;
//    std::cout << "rotation Jacobian LF \n" << tempJacobian.transpose() << std::endl << std::endl;
    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LF, LimbStartIndex::LF,
                                legDof, ee_rot_jac_joints_);
  }

  {
    //RF
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);

    model_.getJacobianRotationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::RF,
                                          loco_m545::RD::BodyNodeEnum::WHEEL,
                                          loco_m545::RD::CoordinateFrameEnum::WORLD);

    //std::cout << " rotation Jacobian RF \n" << tempJacobian.transpose() << std::endl << std::endl;
    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RF, LimbStartIndex::RF,
                                legDof, ee_rot_jac_joints_);
  }

  {
    //LH
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);

    model_.getJacobianRotationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::LH,
                                          loco_m545::RD::BodyNodeEnum::WHEEL,
                                          loco_m545::RD::CoordinateFrameEnum::WORLD);

    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LH, LimbStartIndex::LH,
                                legDof, ee_rot_jac_joints_);
  }

  {
    //RH
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);

    model_.getJacobianRotationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::RH,
                                          loco_m545::RD::BodyNodeEnum::WHEEL,
                                          loco_m545::RD::CoordinateFrameEnum::WORLD);

    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RH, LimbStartIndex::RH,
                                legDof, ee_rot_jac_joints_);
  }

  {
    //BOOM
    tempJacobian.setZero();
    unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);

    model_.getJacobianRotationWorldToBody(tempJacobian, loco_m545::RD::BranchEnum::BOOM,
                                          loco_m545::RD::BodyNodeEnum::ENDEFFECTOR,
                                          loco_m545::RD::CoordinateFrameEnum::WORLD);

    ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::BOOM, LimbStartIndex::BOOM,
                                boomDof, ee_rot_jac_joints_);
  }

}


void M545KinematicModelFull::CalculateTranslationalJacobiansWRTjointsBase(){

  MatrixXd tempJacobian(3, model_.getDofCount());

    {
      //LF
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LF);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::LF,
                                               loco_m545::RD::BodyNodeEnum::WHEEL,
                                               loco_m545::RD::CoordinateFrameEnum::BASE);

  //    std::cout << "Dof count: " << model_.getDofCount() << std::endl;
  //    std::cout << "Jacobian LF \n" << tempJacobian.transpose() << std::endl << std::endl;
      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LF, LimbStartIndex::LF,
                                  legDof, ee_trans_jac_joints_base_);
    }

    {
      //RF
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RF);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::RF,
                                               loco_m545::RD::BodyNodeEnum::WHEEL,
                                               loco_m545::RD::CoordinateFrameEnum::BASE);

      //std::cout << "Jacobian RF \n" << tempJacobian.transpose() << std::endl;
      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RF, LimbStartIndex::RF,
                                  legDof, ee_trans_jac_joints_base_);
    }

    {
      //LH
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::LH);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::LH,
                                               loco_m545::RD::BodyNodeEnum::WHEEL,
                                               loco_m545::RD::CoordinateFrameEnum::BASE);

      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::LH, LimbStartIndex::LH,
                                  legDof, ee_trans_jac_joints_base_);
    }

    {
      //RH
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::RH);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::RH,
                                               loco_m545::RD::BodyNodeEnum::WHEEL,
                                               loco_m545::RD::CoordinateFrameEnum::BASE);

      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::RH, LimbStartIndex::RH,
                                  legDof, ee_trans_jac_joints_base_);
    }

    {
      //BOOM
      tempJacobian.setZero();
      unsigned int ee_id = static_cast<unsigned int>(loco_m545::RD::LimbEnum::BOOM);
      model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, loco_m545::RD::BranchEnum::BOOM,
                                               loco_m545::RD::BodyNodeEnum::ENDEFFECTOR,
                                               loco_m545::RD::CoordinateFrameEnum::BASE);

      ExtractJointJacobianEntries(tempJacobian, loco_m545::RD::LimbEnum::BOOM, LimbStartIndex::BOOM,
                                  boomDof, ee_trans_jac_joints_base_);
    }

}


//update base stuff and joints
void M545KinematicModelFull::UpdateModel(const VectorXd &jointAngles, const Vector3d &ypr_base,
                                         const Vector3d &base_position)
{
  //update the base orientation
  excavator_model::ExcavatorState state = model_.getState();
  kindr::Position3D position;
  position.toImplementation() = base_position;

  base_xyz_ = base_position;

  kindr::EulerAnglesYprD euler(ypr_base.z(), ypr_base.y(), ypr_base.x());
  euler.setUnique();
  euler_ypr_ = euler.toImplementation();
  state.setPositionWorldToBaseInWorldFrame(position);
  state.setOrientationBaseToWorld(kindr::RotationQuaternionD(euler));
  model_.setState(state, true, false, false);

  //update the joints
  UpdateModel(jointAngles);

  // calculate the jacobians
  CalculateTranslationalJacobiansWRTjointsAndBaseOrientation();

  CalculateOrientationJacobiansWRTjoints();

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
//  std::cout << bigJacobian.transpose() << std::endl;

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
}

void M545KinematicModelFull::ExtractOrientationJacobianEntries(const MatrixXd &bigJacobian,
                                                               loco_m545::RD::LimbEnum limb,
                                                               EEJac &jacArray)
{

  constexpr unsigned int dim3 = 3;
  constexpr unsigned int offset = 3;  // ofset for the first columns of the jacobian (base linear + angular)

  unsigned int ee_id = static_cast<unsigned int>(limb);

  for (unsigned int i = 0; i < dim3; ++i)
    for (unsigned int j = 0; j < dim3; ++j) {
      jacArray.at(ee_id).coeffRef(i, j) = bigJacobian(i, offset + j);

    }

//  Eigen::Matrix3d mat = angularVelocity3eulerDerivativesMat(euler_ypr_);
//  jacArray.at(ee_id) = jacArray.at(ee_id) * mat.sparseView();
}

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetTranslationalJacobiansWRTjoints()
{
  return ee_trans_jac_joints_;
}

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetTranslationalJacobianWRTbasePosition()
{
  return ee_trans_jac_base_position_;
}

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetTranslatinalJacobianWRTbaseOrientation()
{
  return ee_trans_jac_base_orientation_;
}

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetOrientationJacobiansWRTjoints()
{
  return ee_rot_jac_joints_;
}

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetOrientationJacobiansWRTbaseOrientation()
{
  return ee_rot_jac_base_orientation_;
}

Eigen::Matrix3d M545KinematicModelFull::angularVelocity3eulerDerivativesMat(const Vector3d &ypr)
{

  using namespace std;
  Eigen::Matrix3d mat;
  double x = ypr.x();
  double y = ypr.y();
  double z = ypr.z();

  mat << 1, sin(x) * sin(y) / cos(y), cos(x) * sin(y) / cos(y), 0, cos(x), -sin(x), 0, sin(x)
      / cos(y), cos(x) / cos(y);

  return mat;

}

const M545KinematicModelFull::EEJac &M545KinematicModelFull::GetTranslationalJacobiansWRTjointsBase() {
  return ee_trans_jac_joints_base_;
}

}
/*namespace*/
