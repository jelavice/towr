/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/range_of_motion_constraint_joints.h>

namespace towr {

RangeOfMotionConstraintJoints::RangeOfMotionConstraintJoints(
    const KinematicModel::Ptr& robot_model, double T, double dt, const EE& ee,
    const SplineHolder& spline_holder)
    : RangeOfMotionConstraint(robot_model, T, dt, ee, spline_holder)
{
//  base_linear_ = spline_holder.base_linear_;
//  base_angular_ = EulerConverter(spline_holder.base_angular_);
//  ee_motion_ = spline_holder.ee_motion_.at(ee);
//
//  max_deviation_from_nominal_ = model->GetMaximumDeviationFromNominal();
//  nominal_ee_pos_B_ = model->GetNominalStanceInBase().at(ee);
//  ee_ = ee;
//
//  SetRows(GetNumberOfNodes() * k3D);
}

void RangeOfMotionConstraintJoints::UpdateConstraintAtInstance(double t, int k,
                                                                    VectorXd& g) const
{

}
void RangeOfMotionConstraintJoints::UpdateBoundsAtInstance(double t, int k, VecBound&) const
{
}
void RangeOfMotionConstraintJoints::UpdateJacobianAtInstance(double t, int k, std::string,
                                                                  Jacobian&) const
{
}

int RangeOfMotionConstraintJoints::GetRow(int node, int dimension) const
{
}

} /*namespace*/
