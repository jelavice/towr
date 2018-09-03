/*
 * nlp_formulation_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/nlp_formulation.h>
#include <towr/parameters_extended.h>

namespace towr {

class NlpFormulationExtended : public NlpFormulation
{

 public:

  using Base = NlpFormulation;
  using VariableSetName = ParametersExtended::VariableSetName;
  using Params = ParametersExtended;

  NlpFormulationExtended();

  std::vector<NodesVariables::Ptr> MakeJointVariables() const;

  VariablePtrVec GetVariableSets(SplineHolder& spline_holder) final;


 private:

  void GetVariableSet(VariableSetName var_set, SplineHolder &spline_holder, VariablePtrVec &vars);

};

} /* namespace */
