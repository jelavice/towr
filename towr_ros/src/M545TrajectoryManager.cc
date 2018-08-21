/*
 * M545TrajecotryManager.cc
 *
 *  Created on: Aug 20, 2018
 *      Author: jelavice
 */

#include <towr_ros/M545TrajectoryManager.h>
#include <towr/variables/euler_converter.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>



namespace towr {

M545TrajectoryManager::M545TrajectoryManager(const HeightMap *terrain)
    : terrain_(terrain)
{

}

M545TrajectoryManager::XppVecCartesian M545TrajectoryManager::GetTrajectoryCartesian(
    const SplineHolder &solution)
{

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
    t += visualization_dt_;

  }

  return trajectory;
}

//todo fix copy paste
M545TrajectoryManager::XppVecJoints M545TrajectoryManager::GetTrajectoryJoints(
    const SplineHolder &solution)
{

  //todo not use the joint class at all it is restrictive for what I need
  XppVecJoints trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  // find the trajecotry with
  int maxJointNum = 0;
  for (int i = 0; i < solution.joint_motion_.size(); ++i) {
    if (solution.joint_motion_.at(i)->GetPoint(0).p().size() > maxJointNum)
      maxJointNum = solution.joint_motion_.at(i)->GetPoint(0).p().size();
  }

  while (t <= T + 1e-5) {
    int n_ee = solution.joint_motion_.size();
    //
    xpp::RobotStateJoint state(n_ee, maxJointNum);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    Eigen::VectorXd all_ee_joints(n_ee * maxJointNum), all_zeros(n_ee * maxJointNum);
    all_zeros.setZero();
    all_ee_joints.setZero();
    int count = 0;

    // copy the contacts and stack all joints ito a vector
    for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);

      // now actually copy the useful values and skip the excess num of joints
      // damn you alex and your stupid joint class
      Eigen::VectorXd temp = solution.joint_motion_.at(ee_towr)->GetPoint(t).p();
      for (int i = 0; i < maxJointNum; ++i)
        if (i < temp.size())
          all_ee_joints(count++) = temp(i);
        else
          ++count;

    }

    state.q_.SetFromVec(all_ee_joints);

    // set the velocities and the torques to zero
    state.qd_.SetFromVec(all_zeros);
    state.qdd_.SetFromVec(all_zeros);
    state.torques_.SetFromVec(all_zeros);

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;

  }

  return trajectory;
}

void M545TrajectoryManager::SaveTrajectoryInRosbagCartesian(rosbag::Bag& bag,
                                                            const std::string& topic,
                                                            const SplineHolder &solution)
{

  auto traj = GetTrajectoryCartesian(solution);

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
        Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
        terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
        terrain_msg.friction_coeff = terrain_->GetFrictionCoeff();
      }

      bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
    }

}

//todo fix code copy paste
void M545TrajectoryManager::SaveTrajectoryInRosbagJoints(rosbag::Bag& bag,
                                                            const std::string& topic,
                                                            const SplineHolder &solution)
{

  auto traj = GetTrajectoryJoints(solution);

  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6);  // t=0.0 throws ROS exception

    xpp_msgs::RobotStateJoint msg;

    msg.base = xpp::Convert::ToRos(state.base_);

        //convert the joints too
    Eigen::VectorXd jointVec = state.q_.ToVec();
    for (int i = 0; i < jointVec.size(); ++i){
      msg.joint_state.name.push_back("");
      msg.joint_state.position.push_back(jointVec(i));
      msg.joint_state.velocity.push_back(0.0);
      msg.joint_state.effort.push_back(0.0);

    }

    //copy the joint contact info
    for (int i =0; i < state.ee_contact_.GetEECount(); ++i)
      msg.ee_contact.push_back(state.ee_contact_.at(i));


    bag.write(topic, timestamp, msg);

    //todo just stick the trajectory in the the cartesian state this shit with joints is annoying

//    xpp_msgs::TerrainInfo terrain_msg;
//    for (auto ee : state.ee_motion_.ToImpl()) {
//      Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
//      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
//      terrain_msg.friction_coeff = terrain_->GetFrictionCoeff();
//    }
//
//    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }

}

} /* namespace */

