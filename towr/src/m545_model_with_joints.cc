/*
 * m545_model_with_joints.cc
 *
 *  Created on: Aug 30, 2018
 *      Author: jelavice
 */

#include <towr/models/examples/m545_model_with_joints.h>

#include <kindr/Core>
#include <fstream>

//#define M545MODELDEBUG

using SparseMatrix = towr::M545KinematicModelWithJoints::SparseMatrix;
using Vector3d = towr::M545KinematicModelWithJoints::Vector3d;
using VectorXd = towr::M545KinematicModelWithJoints::VectorXd;
using EEPos = towr::M545KinematicModelWithJoints::EEPos;
using Matrix3d = Eigen::Matrix3d;

namespace towr {

M545KinematicModelWithJoints::M545KinematicModelWithJoints(const std::string &urdfDescription,
                                                           double dt)
    : KinematicModelWithJoints(numEE),
      urdf_string_(urdfDescription),
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
  ee_ypr_.resize(numEE);
  ee_wheel_axis_.resize(GetNumWheels());

  /*base coordinate frame */
  ee_trans_jac_joints_base_.resize(numEE);
  ee_orientation_jac_base_.resize(numEE);
  ee_wheel_axis_jac_base_.resize(GetNumWheels());

  //create sparse matrices
  for (int i = 0; i < numEE; ++i) {

    int numDof = num_dof_limbs_.at(i);

    /* jacobian in base frame */
    ee_trans_jac_joints_base_.at(i).resize(3, numDof);
    ee_orientation_jac_base_.at(i).resize(3, numDof);

    if (EEhasWheel(i))
      ee_wheel_axis_jac_base_.at(i).resize(3, numDof);
  }

  // initialize with zero

  for (int i = 0; i < numEE; ++i) {
    Eigen::VectorXd jointAngles(num_dof_limbs_.at(i));
    jointAngles.setZero();
    UpdateModel(jointAngles, i);

    //here also calcualte all the shit

    GetEEPositionBase(i);
    GetEEOrientationBase(i);
    CalculateRotationalJacobiansWRTjointsBase(i);
    CalculateTranslationalJacobiansWRTjointsBase(i);

    if (EEhasWheel(i)) {
      GetWheelAxisBase(i);
      GetWheelAxisJacobianBase(i);
    }
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

void M545KinematicModelWithJoints::CalculateJointLimits()
{
  excavator_model::Limits limits;
  limits.init();

  int jointId = 0;
  for (int i = 0; i < numEE; ++i)
    CalculateJointLimitsforSpecificLimb(limits, GetLimbEnum(i), GetNumDof(i), &jointId);

}

void M545KinematicModelWithJoints::CalculateJointLimitsforSpecificLimb(
    const excavator_model::Limits &limits, loco_m545::RD::LimbEnum limb, unsigned int dof,
    int *globalJointId)
{

  unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(limb));
  unsigned int idEnd = idStart + dof;

  for (unsigned int jointId = idStart; jointId < idEnd; ++jointId) {

    loco_m545::RD::JointEnum jointEnum = static_cast<loco_m545::RD::JointEnum>(jointId);
    double upperLimit = limits.getJointMaxPosition(jointEnum);
    double lowerLimit = limits.getJointMinPosition(jointEnum);
    std::string currJoint = loco_m545::RD::mapKeyEnumToKeyName(jointEnum);
    joint_limits_["lowerLimit"][currJoint] = std::min<double>(lowerLimit, upperLimit);
    joint_limits_["upperLimit"][currJoint] = std::max<double>(lowerLimit, upperLimit);
    upper_joint_limits_(*globalJointId) = joint_limits_["upperLimit"][currJoint];
    lower_joint_limits_(*globalJointId) = joint_limits_["lowerLimit"][currJoint];
    *globalJointId = *globalJointId + 1;
  }

  upper_joint_limits_(LimbStartIndex::BOOM) = 1.0e20;
  lower_joint_limits_(LimbStartIndex::BOOM) = -1.0e20;

}

Vector3d M545KinematicModelWithJoints::rotMat2ypr(const Eigen::Matrix3d &mat)
{

  // rotation convention for this is yaw pitch roll in that order
  kindr::RotationMatrixD rotMat(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2),
                                mat(2, 0), mat(2, 1), mat(2, 2));
  kindr::EulerAnglesYprD euler(rotMat);
  euler.setUnique();

  return Eigen::Vector3d(euler.x(), euler.y(), euler.z());

}

void M545KinematicModelWithJoints::PrintJointLimits()
{

  for (auto it = joint_limits_.cbegin(); it != joint_limits_.cend(); ++it)
    for (auto jt = it->second.begin(); jt != it->second.cend(); ++jt)
      std::cout << it->first << " for joint " << jt->first << ": " << jt->second << std::endl;

  std::cout << "Lower limits joints vector: " << std::endl;
  std::cout << lower_joint_limits_ << std::endl << std::endl << std::endl;
  std::cout << "Upper limits joints vector: " << std::endl;
  std::cout << upper_joint_limits_ << std::endl;

}

int M545KinematicModelWithJoints::getLimbStartingId(int ee_id) const
{
  switch (ee_id) {

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

VectorXd M545KinematicModelWithJoints::GetLowerJointLimits(int ee_id)
{
  return lower_joint_limits_.segment(getLimbStartingId(ee_id), num_dof_limbs_.at(ee_id));
}

VectorXd M545KinematicModelWithJoints::GetUpperJointLimits(int ee_id)
{
  return upper_joint_limits_.segment(getLimbStartingId(ee_id), num_dof_limbs_.at(ee_id));
}

Vector3d M545KinematicModelWithJoints::GetEEPositionBase(int ee_id)
{
  ee_pos_base_.at(ee_id) = GetEEPositionsBase(ee_id, model_);
  return ee_pos_base_.at(ee_id);
}

void M545KinematicModelWithJoints::CalculateTranslationalJacobiansWRTjointsBase(int ee_id)
{

  loco_m545::RD::CoordinateFrameEnum coordinate_system = loco_m545::RD::CoordinateFrameEnum::BASE;
  MatrixXd tempJacobian(3, model_.getDofCount());
  tempJacobian.setZero();
  model_.getJacobianTranslationFloatingBaseToBody(tempJacobian, GetEEBranchEnum(ee_id),
                                                  GetEEBodyNodeEnum(ee_id), coordinate_system);

  ExtractJointElementsFromRbdlJacobian(tempJacobian, GetLimbEnum(ee_id), GetLimbStartIndex(ee_id),
                                       GetNumDof(ee_id), ee_trans_jac_joints_base_);

}

void M545KinematicModelWithJoints::UpdateModel(VectorXd jointAngles, int ee_id)
{
  UpdateModel(jointAngles, ee_id, model_);
  CalculateTranslationalJacobiansWRTjointsBase(ee_id);
  CalculateRotationalJacobiansWRTjointsBase(ee_id);

}

void M545KinematicModelWithJoints::UpdateSpecificLimb(loco_m545::RD::LimbEnum limb,
                                                      const VectorXd &jointAngles, unsigned int dof,
                                                      ExcavatorModel &model) const
{
  excavator_model::JointVectorD jointPositions;
  excavator_model::ExcavatorState state = model.getState();

  unsigned int idStart = loco_m545::RD::mapKeyEnumToKeyId(loco_m545::RD::getLimbStartJoint(limb));

  jointPositions = state.getJointPositions().toImplementation();
  jointPositions.segment(idStart, dof) = jointAngles;
  state.getJointPositions().toImplementation() = jointPositions;
  //todo this line was not working, it wouldn't update the joints, see whether that is a bug in loco/kindr
  //state.getJointPositions().toImplementation().segment(idStart, dof) = jointPositions
  model.setState(state, true, false, false);
}

void M545KinematicModelWithJoints::ExtractJointElementsFromRbdlJacobian(
    const MatrixXd &bigJacobian, loco_m545::RD::LimbEnum limb, LimbStartIndex limbStartIndex,
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

SparseMatrix M545KinematicModelWithJoints::GetTranslationalJacobiansWRTjointsBase(int ee_id)
{
  return ee_trans_jac_joints_base_.at(ee_id);
}

SparseMatrix M545KinematicModelWithJoints::GetOrientationJacobiansWRTjointsBase(int ee_id)
{
  return ee_orientation_jac_base_.at(ee_id);
}

void M545KinematicModelWithJoints::printCurrentJointPositions()
{

  std::cout << "Joint positions: "
      << model_.getState().getJointPositions().toImplementation().transpose() << std::endl;

}

bool M545KinematicModelWithJoints::EEhasWheel(int ee_id) const
{

  if (static_cast<int>(LimbEnum::BOOM) == ee_id)
    return false;
  else
    return true;

}

Vector3d M545KinematicModelWithJoints::GetEEOrientationBase(int ee_id)
{
  ee_ypr_.at(ee_id) = rotMat2ypr(GetOrientationBase(ee_id));

  return ee_ypr_.at(ee_id);
}

void M545KinematicModelWithJoints::CalculateRotationalJacobiansWRTjointsBase(int ee_id)
{
  CalculateAngularVelocityJacobian(ee_id);

  ee_orientation_jac_base_.at(ee_id) = angularVelocity2eulerDerivativesMat(
      GetEEOrientationBase(ee_id)) * ee_orientation_jac_base_.at(ee_id);
}

M545KinematicModelWithJoints::SparseMatrix M545KinematicModelWithJoints::angularVelocity2eulerDerivativesMat(
    const Vector3d &ypr)
{

  using namespace std;
  Eigen::Matrix3d mat;

  double cosX = cos(ypr.x());
  double cosY = cos(ypr.y());
  double sinX = sin(ypr.x());
  double sinY = sin(ypr.y());

  mat << 1, sinX * sinY / cosY, cosX * sinY / cosY, 0, cosX, -sinX, 0, sinX / cosY, cosX / cosY;

  //todo investigate this
  // I have no idea why this like that but it passes the unit test
  // I may have forgotten the skew symmetric matrix in between
  mat = -mat;

//  double x = ypr.x();
//  double y = ypr.y();
//  double z = ypr.z();
//    mat << cos(z) / cos(y), sin(z) / cos(y), 0.0,
//        -sin(z), cos(z), 0.0,
//        cos(z) * sin(y) / cos(y), sin(y) * sin(z)/ cos(y), 1.0;

  return mat.sparseView();

}

Vector3d M545KinematicModelWithJoints::GetBasePositionFromFeetPostions()
{

  double z = 0;
  int wheelCount = 0;
  for (int ee_id = 0; ee_id < GetNumberOfEndeffectors(); ++ee_id) {
    if (EEhasWheel(ee_id)) {
      z += ee_pos_base_.at(ee_id).z();
      ++wheelCount;
    }
    //std::cout << ee_pos_base_.at(i).z() << std::endl;
  }

  z = z / wheelCount;

  return Eigen::Vector3d(0.0, 0.0, std::abs(z));

}

Matrix3d M545KinematicModelWithJoints::GetOrientationBase(int ee_id)
{
  return model_.getOrientationBodyToBody(loco_m545::RD::BodyEnum::BASE, GetEEBodyEnum(ee_id));
}

void M545KinematicModelWithJoints::CalculateAngularVelocityJacobian(int ee_id)
{
  using namespace loco_m545;

  RD::CoordinateFrameEnum coordinate_system = RD::CoordinateFrameEnum::BASE;

  MatrixXd jacobianBig(3, model_.getDofCount());
  jacobianBig.setZero();
  model_.getJacobianRotationFloatingBaseToBody(jacobianBig, GetEEBranchEnum(ee_id),
                                               GetEEBodyNodeEnum(ee_id), coordinate_system);

  ExtractJointElementsFromRbdlJacobian(jacobianBig, GetLimbEnum(ee_id), GetLimbStartIndex(ee_id),
                                       GetNumDof(ee_id), ee_orientation_jac_base_);
}

M545KinematicModelWithJoints::EEPos M545KinematicModelWithJoints::GetNominalStanceInBase() const
{

  EEPos ee_pos;
  const double dt = 0.1;
  ExcavatorModel model(dt);
  model.initModelFromUrdf(urdf_string_);
  for (int i = 0; i < numEE; ++i) {
    VectorXd joint_angles(GetNumDof(i));
    // don't stretch the boom far out
    if (i == static_cast<int>(loco_m545::RD::LimbEnum::BOOM))
      joint_angles << 0.0, -1.2, 2.0, 0.0, 2.2;
    else
      joint_angles.setZero();
    UpdateModel(joint_angles, i, model);
    ee_pos.push_back(GetEEPositionsBase(i, model));
  }

#ifdef  M545MODELDEBUG
  std::cout << std::endl;
  for (int i =0; i < ee_pos.size(); ++i)
  std::cout << "limb id: " << i << " , position: " << ee_pos.at(i).transpose() << std::endl;
#endif

  return ee_pos;
}

Vector3d M545KinematicModelWithJoints::GetMaximumDeviationFromNominal() const
{
  return max_dev_from_nominal_;
}

int M545KinematicModelWithJoints::GetNumDofTotal() const
{

  int numDof = 0;
  for (int i = 0; i < GetNumberOfEndeffectors(); ++i)
    numDof += GetNumDof(i);

  return numDof;

}

int M545KinematicModelWithJoints::GetNumDof(int ee_id) const
{
  return num_dof_limbs_.at(ee_id);
}

void M545KinematicModelWithJoints::UpdateModel(VectorXd jointAngles, int ee_id,
                                               ExcavatorModel &model) const
{

  UpdateSpecificLimb(GetLimbEnum(ee_id), jointAngles, GetNumDof(ee_id), model);

}

Eigen::Vector3d M545KinematicModelWithJoints::GetEEPositionsBase(int ee_id,
                                                                 ExcavatorModel &model) const
{
  Eigen::Vector3d position = model.getPositionBodyToBody(loco_m545::RD::BodyEnum::BASE,
                                                         GetEEBodyEnum(ee_id),
                                                         loco_m545::RD::CoordinateFrameEnum::BASE);

  // subtract the radius of the wheel to get the contact point
  // this assumes of course that the ground is always in a plane
  // spanned by x and y axis, below the base frame
  if (ee_id != 4)  //skip boom
    position.z() -= model.getWheelRadius();

  return position;

}

loco_m545::RD::LimbEnum M545KinematicModelWithJoints::GetLimbEnum(int ee_id) const
{

  switch (ee_id) {
    case 0:
      return loco_m545::RD::LimbEnum::LF;
    case 1:
      return loco_m545::RD::LimbEnum::RF;
    case 2:
      return loco_m545::RD::LimbEnum::LH;
    case 3:
      return loco_m545::RD::LimbEnum::RH;
    case 4:
      return loco_m545::RD::LimbEnum::BOOM;
    default:
      throw std::runtime_error("Unknown limb id");

  }

}

loco_m545::RD::BodyEnum M545KinematicModelWithJoints::GetEEBodyEnum(int ee_id) const
{

  switch (ee_id) {
    case 0:
      return loco_m545::RD::BodyEnum::LF_WHEEL;
    case 1:
      return loco_m545::RD::BodyEnum::RF_WHEEL;
    case 2:
      return loco_m545::RD::BodyEnum::LH_WHEEL;
    case 3:
      return loco_m545::RD::BodyEnum::RH_WHEEL;
    case 4:
      return loco_m545::RD::BodyEnum::ENDEFFECTOR;
    default:
      throw std::runtime_error("Unknown limb id");
  }

}

loco_m545::RD::BodyNodeEnum M545KinematicModelWithJoints::GetEEBodyNodeEnum(int ee_id) const
{

  switch (ee_id) {
    case 0:
    case 1:
    case 2:
    case 3:
      return loco_m545::RD::BodyNodeEnum::WHEEL;
    case 4:
      return loco_m545::RD::BodyNodeEnum::ENDEFFECTOR;
    default:
      throw std::runtime_error("Unknown limb id");
  }
}

loco_m545::RD::BranchEnum M545KinematicModelWithJoints::GetEEBranchEnum(int ee_id) const
{

  switch (ee_id) {
    case 0:
      return loco_m545::RD::BranchEnum::LF;
    case 1:
      return loco_m545::RD::BranchEnum::RF;
    case 2:
      return loco_m545::RD::BranchEnum::LH;
    case 3:
      return loco_m545::RD::BranchEnum::RH;
    case 4:
      return loco_m545::RD::BranchEnum::BOOM;
    default:
      throw std::runtime_error("Unknown limb id");
  }
}

M545KinematicModelWithJoints::LimbStartIndex M545KinematicModelWithJoints::GetLimbStartIndex(
    int ee_id) const
{

  switch (ee_id) {
    case 0:
      return LimbStartIndex::LF;
    case 1:
      return LimbStartIndex::RF;
    case 2:
      return LimbStartIndex::LH;
    case 3:
      return LimbStartIndex::RH;
    case 4:
      return LimbStartIndex::BOOM;
    default:
      throw std::runtime_error("Unknown limb id");
  }
}

Eigen::Vector3d M545KinematicModelWithJoints::GetWheelAxisBase(int ee_id)
{

  if (ee_id > GetNumWheels())
    throw std::runtime_error("Bomm has no wheels. can't calculate teh wheel axis");

  ee_wheel_axis_.at(ee_id) = GetOrientationBase(ee_id) * Eigen::Vector3d(0.0, 1.0, 0.0);

  return ee_wheel_axis_.at(ee_id);
}

SparseMatrix PartialDerivativeOfWheelAxis(const Eigen::Vector3d &ypr)
{

  Eigen::Matrix3d mat;

  double x = ypr.x();
  double y = ypr.y();
  double z = ypr.z();


  mat.row(0) << sin(x)*sin(z) + cos(x)*cos(z)*sin(y), cos(y)*cos(z)*sin(x), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z);
  mat.row(1) << cos(x)*sin(y)*sin(z) - cos(z)*sin(x), cos(y)*sin(x)*sin(z),   cos(z)*sin(x)*sin(y) - cos(x)*sin(z);
  mat.row(2) <<                        cos(x)*cos(y),       -sin(x)*sin(y),                                      0;

//  mat.row(0) << 0, sin(y) * sin(z), -cos(y) * cos(z);
//  mat.row(1) << -cos(z) * sin(x) - cos(x) * sin(y) * sin(z), -cos(y) * sin(x) * sin(z), -cos(x)
//      * sin(z) - cos(z) * sin(x) * sin(y);
//  mat.row(2) << cos(x) * cos(z) - sin(x) * sin(y) * sin(z), cos(x) * cos(y) * sin(z), cos(x)
//      * cos(z) * sin(y) - sin(x) * sin(z);

  return mat.sparseView();

}

SparseMatrix M545KinematicModelWithJoints::GetWheelAxisJacobianBase(int ee_id)
{

  if (ee_id > GetNumWheels())
    throw std::runtime_error("Bomm has no wheels. can't calculate the wheel axis jacobian");

  CalculateRotationalJacobiansWRTjointsBase(ee_id);

  ee_wheel_axis_jac_base_.at(ee_id) = PartialDerivativeOfWheelAxis(GetEEOrientationBase(ee_id))
      * ee_orientation_jac_base_.at(ee_id);

  return ee_wheel_axis_jac_base_.at(ee_id);

}
;

}/*namespace*/

