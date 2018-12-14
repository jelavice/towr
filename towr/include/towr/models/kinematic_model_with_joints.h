/*
 * kinematic_model_with_joints.h
 *
 *  Created on: Aug 30, 2018
 *      Author: jelavice
 */

#pragma once

#include "kinematic_model.h"
#include <Eigen/SparseCore>
#include <kindr/Core>

namespace towr {

class KinematicModelWithJoints : public KinematicModel
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<KinematicModelWithJoints>;
  using VectorXd = KinematicModel::VectorXd;
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using EEJac = std::vector<SparseMatrix>;
  using MatrixXd = Eigen::MatrixXd;

  KinematicModelWithJoints(int n_ee)
      : KinematicModel(n_ee)
  {
  }

  virtual ~KinematicModelWithJoints() = default;

  virtual int GetNumDof(int ee_id) const = 0;

  virtual int GetNumDofTotal() const
  {
    return -1;
  }

  virtual bool EEhasWheel(int limbId) const{
    throw std::runtime_error("Method EEhasWheel not implemented");
    return false;
  }

  virtual void UpdateModel(VectorXd jointAngles, int limbId) = 0;
  virtual VectorXd GetLowerJointLimits(int limbId) = 0;
  virtual VectorXd GetUpperJointLimits(int limbId) = 0;

  /* base methods *
   * */
  virtual Eigen::Vector3d GetEEPositionBase(int ee_id) = 0;
  virtual Eigen::Vector3d GetEEOrientationBase(int ee_id) = 0;
  virtual Eigen::Vector3d GetBasePositionFromFeetPostions() = 0;
  virtual SparseMatrix GetTranslationalJacobiansWRTjointsBase(int ee_id) = 0;
  virtual SparseMatrix GetOrientationJacobiansWRTjointsBase(int ee_id) = 0;



  //todo rename this into get wheel axis
  virtual Eigen::Vector3d GetWheelAxisBase(int ee_id)
  {
    throw std::runtime_error("Not implemented the get wheel axis base");
  }

  virtual SparseMatrix GetWheelAxisJacobianBase(int ee_id)
  {
    throw std::runtime_error("Not implemented the get wheel axis jacobian base");
  }

  virtual Eigen::Matrix3d GetRotationBaseToWheel(int ee_id){
    throw std::runtime_error("Not implemented the get rotation matrix base to wheel method");
  }

  virtual SparseMatrix GetDerivOfRotVecMult(const Eigen::Vector3d &vector, int ee_id, bool compute_inverse){
    throw std::runtime_error("NOt implemented Get derivative of rotation matrix multiplying a vector");
  }

};

} /* namespace towr */
