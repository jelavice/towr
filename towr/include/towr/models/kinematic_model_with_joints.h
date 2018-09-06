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

  virtual int GetNumDofTotal() const
  {
    return -1;
  }

  virtual int GetNumDof(int ee_id) const = 0;

  virtual VectorXd GetLowerJointLimits(int limbId) = 0;
  virtual VectorXd GetUpperJointLimits(int limbId) = 0;

  virtual void UpdateModel(VectorXd jointAngles, int limbId) = 0;

  /* base methods *
   * */

  virtual Eigen::Vector3d GetEEPositionsBase(int limbId) = 0;
  virtual Eigen::Vector3d GetEEOrientationBase(int limbId) = 0;

  virtual Eigen::Vector3d GetBasePositionFromFeetPostions() = 0;

  virtual SparseMatrix GetTranslationalJacobiansWRTjointsBase(int limbId) = 0;
  virtual SparseMatrix GetOrientationJacobiansWRTjointsBase(int limbId) = 0;



  virtual Eigen::Vector3d GetEEOrientationVectorBase(int limbId, int dim)
  {
    throw std::runtime_error("Not implemented the get ee orientation vector base");
  }

  virtual SparseMatrix GetOrientationVectorJacobianBase(int limbId, int dim)
  {
    throw std::runtime_error("Not implemented the get jacobian ee orientation vector base");
  }

};

} /* namespace towr */
