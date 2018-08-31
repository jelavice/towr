/*
 * parameters_extended.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include "towr/parameters_extended.h"

namespace towr {

ParametersExtended::ParametersExtended(int n_ee)
    : Parameters()
{
  n_ee_ = n_ee;
}

int ParametersExtended::GetEECount() const {
  return n_ee_;
}

void ParametersExtended::MakeTerrainConstraint() {
  constraints_.push_back(Terrain);
}

void ParametersExtended::DeleteAllConstraints(){
  constraints_.clear();
}

void ParametersExtended::SetJointVelocityAndRangeOfMotionConstraint(){
  constraints_.push_back(JointVelocityAndRangeOfMotion);
}

//todo make functions to set all constraints

//make public the ones that alex has

//add joint constraints

//add setters for constraints

//add delete all constraints

//add the parameters that I need e.g. force, joint polynomial duration, joint polynomial consttraint dt



} /* namespace*/

