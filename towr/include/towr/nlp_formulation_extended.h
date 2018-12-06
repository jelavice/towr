/*
 * nlp_formulation_extended.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include <towr/nlp_formulation.h>
#include <towr/parameters_extended.h>
#include <towr/variables/spline_holder_extended.h>
#include "towr/models/kinematic_model_with_joints.h"


namespace towr {

class NlpFormulationExtended : public NlpFormulation
{

 public:

  using Base = NlpFormulation;
  using VariableSetName = ParametersExtended::VariableSetName;
  using Params = ParametersExtended;
  using SplineContainer = SplineHolderExtended;

  NlpFormulationExtended() = default;

  std::vector<NodesVariables::Ptr> MakeJointVariables() const;
  std::vector<NodesVariables::Ptr> MakeEEMotionWithWheelsVariables() const;


  VariablePtrVec GetVariableSets(SplineHolder& spline_holder) override;
  ConstraintPtrVec GetConstraint (Parameters::ConstraintName name,
                             const SplineHolder& s) const override;

  ConstraintPtrVec MakeJointLimitsConstraint(const SplineHolder &s) const;
  ConstraintPtrVec MakeForwardKinematicsConstraint(const SplineHolder &s) const;


 //
 protected:

  std::vector<NodesVariables::Ptr> MakeBaseVariables() const override;
  std::vector<NodesVariablesPhaseBased::Ptr> MakeEndeffectorVariables() const override;



 private:

  void CreateVariableSet(VariableSetName var_set, SplineHolder &spline_holder,
                         VariablePtrVec &vars);

  void CastPointers(KinematicModelWithJoints::Ptr *model, ParametersExtended::Ptr *params ) const;


};

} /* namespace */
