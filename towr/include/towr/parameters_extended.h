/*
 * parameters_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/parameters.h"

namespace towr {

class ParametersExtended : public Parameters {


  ParametersExtended() = delete;
  ParametersExtended(int n_ee);
  ~ParametersExtended() = default;


private:

  int n_ee_;

};

} /* namespace */
