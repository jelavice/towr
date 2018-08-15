/*
 * kinematic_model_joints.h
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */
#pragma once

#include "kinematic_model.h"
#include <Eigen/SparseCore>
#include <kindr/Core>

namespace towr {

class KinematicModelJoints : public KinematicModel
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<KinematicModelJoints>;
  using VectorXd = KinematicModel::VectorXd;
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using EEJac = std::vector<SparseMatrix>;
  using MatrixXd = Eigen::MatrixXd;

  KinematicModelJoints(int n_ee)
      : KinematicModel(n_ee)
  {
  }

  virtual ~KinematicModelJoints() = default;

  virtual int GetNumDofTotal()
  {
    return -1;
  }

  static Eigen::Vector3d rotMat2ypr(const Eigen::Matrix3d &mat)
  {

    // rotation convention for this is yaw pitch roll in that order

    kindr::RotationMatrixD rotMat(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2),
                                  mat(2, 0), mat(2, 1), mat(2, 2));

    kindr::EulerAnglesYprD euler(rotMat);

    euler.setUnique();

    return Eigen::Vector3d(euler.x(), euler.y(), euler.z());

  }

  virtual VectorXd GetLowerJointLimits(int limbId) = 0;
  virtual VectorXd GetUpperJointLimits(int limbId) = 0;

  virtual void UpdateModel(VectorXd jointAngles, int limbId) = 0;

  /* base methods *
   * */

  virtual Eigen::Vector3d GetEEPositionsBase(int limbId) = 0;

  virtual SparseMatrix GetTranslationalJacobiansWRTjointsBase(int limbId) = 0;

};

} /* namespace towr */
