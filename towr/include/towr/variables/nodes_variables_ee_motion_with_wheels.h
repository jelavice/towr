/*
 * nodes_variables_ee_motion_with_wheels.h
 *
 *  Created on: Dec 6, 2018
 *      Author: jelavice
 */

#pragma once

#include "towr/variables/nodes_variables_all.h"
#include "towr/variables/spline_holder.h"
#include "towr/parameters_extended.h"

namespace towr {

struct NonPhaseNodeInfo
{

  NonPhaseNodeInfo() = delete;
  ~NonPhaseNodeInfo() = default;
  NonPhaseNodeInfo(bool isContact, double globalTime)
      : isInContact_(isContact),
        globalTime_(globalTime)
  {
  }
  bool isInContact_ = true;
  double globalTime_;
};

class NodesVariablesEEMotionWithWheels : public NodesVariablesAll
{
 public:

  using Ptr = std::shared_ptr<NodesVariablesEEMotionWithWheels>;
  using Base = NodesVariablesAll;
  using VecTimes = ParametersExtended::VecTimes;

  //todo add here the book keeping
  NodesVariablesEEMotionWithWheels(const SplineHolder &s, const VecTimes &poly_duratinos,
                                   int n_nodes, int n_dim, const std::string &variable_id, int ee);


  const NonPhaseNodeInfo &GetNonPhaseNodeInfoAt(int id);

 private:

  void CalculateContactSchedule(int n_nodes);

  int n_nodes_;
  int ee_;
  PhaseDurations::Ptr phase_durations_;
  VecTimes ee_polynomial_durations_;
  std::vector<NonPhaseNodeInfo> nodes_info_;

};

} /* namespace */
