/*
 * nlp_formulation_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/nlp_formulation.h>
#include <towr/parameters_extended.h>

namespace towr{

class NlpFormulationExtended : public NlpFormulation {

public:

  using Base = NlpFormulation;

  NlpFormulationExtended();

private:


ParametersExtended extended_params_;

std::vector<NodesVariables::Ptr> MakeJointVariables() const;

VariablePtrVec GetVariableSets(SplineHolder& spline_holder) final;


//todo see which functions I have to make virtual in the base class


};


} /* namespace */
