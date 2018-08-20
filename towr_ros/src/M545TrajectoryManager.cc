/*
 * M545TrajecotryManager.cc
 *
 *  Created on: Aug 20, 2018
 *      Author: jelavice
 */

#include <towr_ros/M545TrajectoryManager.h>
#include <towr/variables/euler_converter.h>
#include <xpp_msgs/RobotStateCartesian.h>


namespace towr {

M545TrajectoryManager::XppVecCartesian M545TrajectoryManager::GetTrajectoryCartesian(const SplineHolder &solution){

  XppVecCartesian trajectory;
    double t = 0.0;
    double T = solution.base_linear_->GetTotalTime();

    EulerConverter base_angular(solution.base_angular_);

    while (t <= T + 1e-5) {
      int n_ee = solution.ee_motion_.size();
      xpp::RobotStateCartesian state(n_ee);

      state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

      state.base_.ang.q = base_angular.GetQuaternionBaseToWorld(t);
      state.base_.ang.w = base_angular.GetAngularVelocityInWorld(t);
      state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

      for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
        int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

        state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
        state.ee_motion_.at(ee_xpp) = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
        state.ee_forces_.at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
        //state.wheel_angles_.at(ee_xpp) = solution.ee_wheel_angles_.at(ee_towr)->GetPoint(t).p()(0);
        //std::cout << "ee_xpp: " << ee_xpp << "/" << n_ee << " : " << solution.ee_motion_.at(ee_towr)->GetPoint(t).p().transpose() << std::endl;
      }

      state.t_global_ = t;
      trajectory.push_back(state);
      t += visualization_dt;

    }

    return trajectory;
}


void M545TrajectoryManager::SaveTrajectoryInRosbagCartesian(rosbag::Bag& bag, const XppVecCartesian& traj, const std::string& topic,
                            const HeightMap *terrain){

  for (const auto state : traj) {
      auto timestamp = ::ros::Time(state.t_global_ + 1e-6);  // t=0.0 throws ROS exception

      xpp_msgs::RobotStateCartesian msg;
      msg = xpp::Convert::ToRos(state);

      //todo remove the hack
  //    for (auto wheel_angle : state.wheel_angles_.ToImpl())
  //      msg.wheel_angles.push_back(wheel_angle);

      bag.write(topic, timestamp, msg);

      xpp_msgs::TerrainInfo terrain_msg;
      for (auto ee : state.ee_motion_.ToImpl()) {
        Vector3d n = terrain->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
        terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
        terrain_msg.friction_coeff = terrain->GetFrictionCoeff();
      }

      bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
    }

}



} /* namespace */

