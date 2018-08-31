/*
 * m545_trajectory_manager.h
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/variables/spline_holder.h>
#include <xpp_states/robot_state_cartesian.h>
#include <xpp_states/robot_state_cartesian_plus_joints.h>

#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>

#include <towr/variables/euler_converter.h>

namespace towr {

class M545TrajectoryManager
{

 public:

  using XppVecCartesian = std::vector<xpp::RobotStateCartesian>;
  using XppVecJoints = std::vector<xpp::RobotStateCartesianPlusJoints>;
  using Vector3d = Eigen::Vector3d;

  M545TrajectoryManager(const HeightMap *terrain);
  ~M545TrajectoryManager() = default;

  XppVecCartesian GetTrajectoryCartesian(const SplineHolder &solution);
  void SaveTrajectoryInRosbagCartesian(rosbag::Bag& bag, const std::string& topic,
                                       const SplineHolder &solution);

  XppVecJoints GetTrajectoryJoints(const SplineHolder &solution);
  void SaveTrajectoryInRosbagJoints(rosbag::Bag& bag, const std::string& topic,
                                    const SplineHolder &solution);

 private:

  void FillCartesianState(double t, int n_ee, const SplineHolder &solution,
                          EulerConverter &base_angular, xpp::RobotStateCartesian &state);

  const double visualization_dt_ = 0.02;
  const HeightMap *terrain_ = nullptr;

};

} /* namespace */

