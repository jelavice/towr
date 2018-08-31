/*
 * parameters_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/parameters.h"

namespace towr {

class ParametersExtended : public Parameters
{

 public:

  enum ExtendedConstraintSet
  {
    //thse enums are deifined in the base class
    //todo fix this
  };

  ParametersExtended() = delete;
  ParametersExtended(int n_ee);
  ~ParametersExtended() = default;

  int GetEECount() const final;

  void MakeTerrainConstraint();

  void DeleteAllConstraints();

  void SetJointVelocityAndRangeOfMotionConstraint();

 private:

  int n_ee_;

};

} /* namespace */
