/*
 * M545TrajectoryMangaer.h
 *
 *  Created on: Aug 20, 2018
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
#include <xpp_states/robot_state_joint.h>

#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>




namespace towr {

class M545TrajectoryManager
{

 public:

  using XppVecCartesian = std::vector<xpp::RobotStateCartesian>;
  using XppVectorJoints = std::vector<xpp::RobotStateJoint>;
  using Vector3d = Eigen::Vector3d;

  M545TrajectoryManager() = default;
  ~M545TrajectoryManager() = default;

  XppVecCartesian GetTrajectoryCartesian(const SplineHolder &solution);
  void SaveTrajectoryInRosbagCartesian(rosbag::Bag& bag, const XppVecCartesian& traj,
                                       const std::string& topic, const HeightMap *terrain);

 private:

  const double visualization_dt = 0.1;

};

} /* namespace */
