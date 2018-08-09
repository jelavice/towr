/*
 * kinematic_model_joints.h
 *
 *  Created on: Aug 7, 2018
 *      Author: jelavice
 */
#pragma once

#include "kinematic_model.h"
#include <Eigen/SparseCore>

namespace towr {

class KinematicModelJoints : public KinematicModel
{
 public:
  using Ptr = std::shared_ptr<KinematicModelJoints>;
  using VectorXd = KinematicModel::VectorXd;
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using EEJac = std::vector<SparseMatrix>;

  KinematicModelJoints(const std::vector<int> &limbdofs, int n_ee)
      : KinematicModel(n_ee)
  {
    num_dof_limbs_ = limbdofs;
  }

  virtual ~KinematicModelJoints() = default;

  int GetNumDof(int limbId) override
  {
    return num_dof_limbs_.at(limbId);
  }

  int GetNumDofTotal()
  {

    int numDof = 0;
    for (int i = 0; i < GetNumberOfEndeffectors(); ++i)
      numDof += GetNumDof(i);

    return numDof;

  }

  //todo write here methods for updating all the crap
  virtual const VectorXd GetLowerJointLimits(int limbId) = 0;
  virtual const VectorXd GetUpperJointLimits(int limbId) = 0;

  virtual void UpdateModel(const VectorXd &jointAngles, const Vector3d &ypr_base,
                     const Vector3d &base_position) = 0;


  //todo make all this crap take limb id as an argument
    // these are in the world frame
  virtual const EEPos &GetEEPositionsWorld() = 0;

    // this is in the world frame
  virtual const EEPos &GetEEOrientation() = 0;

    //world frame
  virtual const EEJac &GetTranslationalJacobiansWRTjoints() = 0;

    //world
  virtual const EEJac &GetTranslationalJacobianWRTbasePosition() = 0;

    //world
  virtual const EEJac &GetTranslatinalJacobianWRTbaseOrientation() = 0;

    // dis in the world frame
  virtual const EEJac &GetOrientationJacobiansWRTjoints() = 0;

    // dis in the world frame (dis identity matrix)
  virtual const EEJac &GetOrientationJacobiansWRTbaseOrientation() = 0;

 protected:

  std::vector<int> num_dof_limbs_;

};

} /* namespace towr */
