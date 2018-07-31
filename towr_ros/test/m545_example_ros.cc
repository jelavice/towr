/*
 * m545_example.cc
 *
 *  Created on: Jul 17, 2018
 *      Author: jelavice
 */

#include <towr/terrain/examples/height_map_examples.h>

#include <cmath>

#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_states/robot_state_cartesian.h>
#include <xpp_msgs/RobotStateCartesian.h>

#include <towr/towr.h>
#include <towr_ros/towr_ros.h>

#include <ifopt/ipopt_solver.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/initialization/gait_generator.h>
#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>

#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>

using namespace xpp;
using namespace towr;

using XppVec = std::vector<xpp::RobotStateCartesian>;
using TowrCommandMsg = towr_ros::TowrCommand;
using Vector3d = Eigen::Vector3d;

const double visualization_dt = 0.2;

XppVec GetTrajectory(const SplineHolder &solution)
{

  XppVec trajectory;
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
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt;
//    std::cout << "Here is t: " << t << std::endl;
//    std::cout << "State Base: " << state.base_.lin.p_.transpose() << std::endl;
  }

  return trajectory;
}

void SaveTrajectoryInRosbag(rosbag::Bag& bag, const XppVec& traj,
                                     const std::string& topic, const HeightMap *terrain)
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6);  // t=0.0 throws ROS exception
    std::cout << "ros time stamp: " <<  timestamp.toSec() << std::endl;
    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
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

void printTrajectory(const SplineHolder &x)
{
  // Print out the trajecetory at discrete time samples
  using namespace std;
  cout.precision(2);
  cout << fixed;
  cout << "\n====================\n m545 trajectory: \n====================\n";

  double t = 0.0;
  while (t <= x.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << x.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = x.base_angular_->GetPoint(t).p();
    cout << (rad / M_PI * 180).transpose() << "\t[deg]" << endl;

    cout << "Feet position x,y,z:          \n";
    cout << "LF: " << x.ee_motion_.at(LF)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    cout << "RF: " << x.ee_motion_.at(RF)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    cout << "LH: " << x.ee_motion_.at(LH)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    cout << "RH: " << x.ee_motion_.at(RH)->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Feet velocity x,y,z:          \n";
    cout << "LF: " << x.ee_motion_.at(LF)->GetPoint(t).v().transpose() << "\t[m]" << endl;
    cout << "RF: " << x.ee_motion_.at(RF)->GetPoint(t).v().transpose() << "\t[m]" << endl;
    cout << "LH: " << x.ee_motion_.at(LH)->GetPoint(t).v().transpose() << "\t[m]" << endl;
    cout << "RH: " << x.ee_motion_.at(RH)->GetPoint(t).v().transpose() << "\t[m]" << endl;

    cout << "Contact force x,y,z:          \n";
    cout << "LF: " << x.ee_force_.at(LF)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "RF: " << x.ee_force_.at(RF)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "LH: " << x.ee_force_.at(LH)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "RH: " << x.ee_force_.at(RH)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    cout << "Wheel angle:          \n";
    cout << "LF: " << x.ee_wheel_angles_.at(LF)->GetPoint(t).p() << "\t[N]" << endl;
    cout << "RF: " << x.ee_wheel_angles_.at(RF)->GetPoint(t).p() << "\t[N]" << endl;
    cout << "LH: " << x.ee_wheel_angles_.at(LH)->GetPoint(t).p() << "\t[N]" << endl;
    cout << "RH: " << x.ee_wheel_angles_.at(RH)->GetPoint(t).p() << "\t[N]" << endl;

    bool contact = x.phase_durations_.at(LF)->IsContactPhase(t);
    std::string foot_in_contact = contact ? "yes" : "no";
    cout << " LF Foot in contact:              \t" + foot_in_contact << endl;

    contact = x.phase_durations_.at(RF)->IsContactPhase(t);
    foot_in_contact = contact ? "yes" : "no";
    cout << " RF Foot in contact:              \t" + foot_in_contact << endl;

    contact = x.phase_durations_.at(LH)->IsContactPhase(t);
    foot_in_contact = contact ? "yes" : "no";
    cout << " LH Foot in contact:              \t" + foot_in_contact << endl;

    contact = x.phase_durations_.at(RH)->IsContactPhase(t);
    foot_in_contact = contact ? "yes" : "no";
    cout << " RH Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += 0.5;
  }
}

int main()
{

  // terrain
  auto terrain = std::make_shared<FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the hopper
  RobotModel model(RobotModel::m545);
  Parameters::robot_has_wheels_ = true;

  // set the initial position of the hopper
  BaseState initial_base;
  initial_base.lin.at(towr::kPos) << 0.0, 0.0, 0.95;

  const double x_nominal_b_front = 2.7;
  const double y_nominal_b_front = 1.6;

  const double x_nominal_b_hind = 2.05;
  const double y_nominal_b_hind = 1.26;

  const double z_nominal_b = 0.0;

  towr::TOWR::FeetPos nominal_stance(4);

  nominal_stance.at(towr::QuadrupedIDs::LF) << x_nominal_b_front, y_nominal_b_front, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::RF) << x_nominal_b_front, -y_nominal_b_front, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::LH) << -x_nominal_b_hind, y_nominal_b_hind, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::RH) << -x_nominal_b_hind, -y_nominal_b_hind, z_nominal_b;

  // define the desired goal state of the hopper
  BaseState goal;
  goal.lin.at(towr::kPos) << 0.0, 1.0, 0.95;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  Parameters params;
  // here we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____

  const double duration = 6.0;
  params.SetNumberEEPolynomials(30);
  params.SetDynamicConstraintDt(0.1);
  params.SetRangeOfMotionConstraintDt(0.1);

  //must be called to override the constructor
  params.SetConstraints();

  for (int i = 0; i < 4; ++i) {
    params.ee_phase_durations_.push_back( { duration });
    params.ee_in_contact_at_start_.push_back(true);
  }

  if (Parameters::robot_has_wheels_ == false)
    params.SetSwingConstraint();

  // Pass this information to the actual solver
  TOWR towr;
  towr.SetInitialState(initial_base, nominal_stance);
  towr.SetParameters(goal, params, model, terrain);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("linear_solver", "ma57");
  solver->SetOption("ma57_pre_alloc", 3.0);
  solver->SetOption("max_cpu_time", 80.0);
  //solver->SetOption("jacobian_approximation", "finite-difference-values");

//  solver->SetOption("max_iter", 1);
//  solver->SetOption("derivative_test", "first-order");
//  solver->SetOption("print_level", 4);
//  solver->SetOption("derivative_test_perturbation", 1e-5);
//  solver->SetOption("derivative_test_tol", 1e-3);

  towr.SolveNLP(solver);

  //return 0;

  auto x = towr.GetSolution();

  printTrajectory(x);

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6);  // t=0.0 throws ROS exception
  auto final_trajectory = GetTrajectory(x);
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired, terrain.get());

  bag.close();

}
