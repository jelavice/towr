/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/range_of_motion_constraint_complicated.h>

namespace towr {

void RangeOfMotionConstraintComplicated::UpdateConstraintAtInstance(double t, int k,
                                                                    VectorXd& g) const
{
}
void RangeOfMotionConstraintComplicated::UpdateBoundsAtInstance(double t, int k, VecBound&) const
{
}
void RangeOfMotionConstraintComplicated::UpdateJacobianAtInstance(double t, int k, std::string,
                                                                  Jacobian&) const
{
}

int RangeOfMotionConstraintComplicated::GetRow(int node, int dimension) const
{
}

} /*namespace*/
