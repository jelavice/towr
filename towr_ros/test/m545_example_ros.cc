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

#include <ifopt/ipopt_solver.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/initialization/gait_generator.h>
#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr/nlp_formulation.h>

#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>

#include <towr/models/examples/m545_model.h>

using namespace xpp;
using namespace towr;

using XppVec = std::vector<xpp::RobotStateCartesian>;
using Vector3d = Eigen::Vector3d;

const double visualization_dt = 0.1;

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
      //state.wheel_angles_.at(ee_xpp) = solution.ee_wheel_angles_.at(ee_towr)->GetPoint(t).p()(0);
      //std::cout << "ee_xpp: " << ee_xpp << "/" << n_ee << " : " << solution.ee_motion_.at(ee_towr)->GetPoint(t).p().transpose() << std::endl;
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt;

  }

  return trajectory;
}

void SaveTrajectoryInRosbag(rosbag::Bag& bag, const XppVec& traj, const std::string& topic,
                            const HeightMap *terrain)
{
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
    cout << "Base linear velocity x,y,z:   \t";
    cout << x.base_linear_->GetPoint(t).v().transpose() << "\t[m]" << endl;

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

    //not used for joints
//    cout << "Wheel angle:          \n";
//    cout << "LF: " << x.ee_wheel_angles_.at(LF)->GetPoint(t).p() << "\t[N]" << endl;
//    cout << "RF: " << x.ee_wheel_angles_.at(RF)->GetPoint(t).p() << "\t[N]" << endl;
//    cout << "LH: " << x.ee_wheel_angles_.at(LH)->GetPoint(t).p() << "\t[N]" << endl;
//    cout << "RH: " << x.ee_wheel_angles_.at(RH)->GetPoint(t).p() << "\t[N]" << endl;

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

void setParameters(NlpFormulation &formulation, double duration, const std::string &urdfDescription)
{


  // Kinematic limits and dynamic parameters of the hopper
    constexpr double dt = 0.01;
    Parameters::robot_has_wheels_ = true;
    Parameters::use_joint_formulation_ = true;
    int numEE = 0;

  if (Parameters::use_joint_formulation_)
    formulation.model_ = RobotModel(RobotModel::m545full, urdfDescription, dt);
  else
    formulation.model_ = RobotModel(RobotModel::m545);

  // set the initial position of the excavator
  BaseState initial_base;
  auto &base_init_pos = formulation.initial_base_.lin.at(towr::kPos);
  auto &base_init_orientation = formulation.initial_base_.ang.at(towr::kPos);
  Parameters& params = formulation.params_;




  double baseHeight = 0.95;
  base_init_pos << 0.0, 0.0, baseHeight;
  base_init_orientation << 0.0, 0.0, 0.0;

  if (Parameters::use_joint_formulation_) {

    auto model = std::dynamic_pointer_cast<M545KinematicModelFull>(
        formulation.model_.kinematic_model_);

    if (model == nullptr)
      throw std::runtime_error(
          "Cannot do the dynamic cast from kinematic model to kinematic model full");

    // joints will be set to zero by default

    int numEE = model->numEE;

    auto &nominal_stance = formulation.initial_ee_W_;
    nominal_stance.resize(numEE);

    //then compute the ee positions

    for (int i = 0; i < numEE; ++i) {
      Eigen::Vector3d stance = model->GetEEPositionsBase(i);
      stance.z() = 0;  // this sortof sets it to world frame
      nominal_stance.at(i) = stance;
      //std::cout << nominal_stance.at(i).transpose() << std::endl;
    }

    params.SetEECount(numEE);

    for (int i = 0; i < 4; ++i) {
      params.ee_phase_durations_.push_back( { duration });
      params.ee_in_contact_at_start_.push_back(true);
    }

    //the boom
    params.ee_phase_durations_.push_back( { duration });
    params.ee_in_contact_at_start_.push_back(false);

    baseHeight = model->GetBasePosition().z();
    base_init_pos << 0.0, 0.0, baseHeight;
    base_init_orientation << 0.0, 0.0, 0.0;

  } else {  // if one ain't using joint formulations

    params.SetEECount(4);

    //also in the world frame
    const double x_nominal_b_front = 2.7;
    const double y_nominal_b_front = 1.6;

    const double x_nominal_b_hind = 2.05;
    const double y_nominal_b_hind = 1.26;

    const double z_nominal_b = 0.0;

    auto &nominal_stance = formulation.initial_ee_W_;
    nominal_stance.resize(4);

    nominal_stance.at(towr::QuadrupedIDs::LF) << x_nominal_b_front, y_nominal_b_front, z_nominal_b;
    nominal_stance.at(towr::QuadrupedIDs::RF) << x_nominal_b_front, -y_nominal_b_front, z_nominal_b;
    nominal_stance.at(towr::QuadrupedIDs::LH) << -x_nominal_b_hind, y_nominal_b_hind, z_nominal_b;
    nominal_stance.at(towr::QuadrupedIDs::RH) << -x_nominal_b_hind, -y_nominal_b_hind, z_nominal_b;

    for (int i = 0; i < 4; ++i) {
      params.ee_phase_durations_.push_back( { duration });
      params.ee_in_contact_at_start_.push_back(true);
    }

  }

  formulation.final_base_.lin.at(towr::kPos) << 0.0, 0.0, baseHeight;


}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "m545_example_node");
  ros::NodeHandle nh;

  //get the excavator model
  std::string urdfDescription;
  nh.getParam("romo_mm_description", urdfDescription);

  NlpFormulation formulation;
  Parameters& params = formulation.params_;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);


  const double duration = 1.0;

  int num_polys = static_cast<int>(duration / 0.25);

  params.SetNumberEEPolynomials(num_polys);
  params.SetDynamicConstraintDt(0.2);
  params.SetRangeOfMotionConstraintDt(0.2);
  params.SetPolynomialDurationBase(0.1);
  params.SetPolynomialDurationJoints(0.2);

  setParameters(formulation, duration, urdfDescription);

  // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos).x() = 1.0;
  formulation.final_base_.lin.at(towr::kPos).y() = 1.0;

  std::cout << "Initial position of the base "
            << formulation.initial_base_.lin.at(towr::kPos).transpose() << std::endl;
  std::cout << "Final position of the base "
            << formulation.final_base_.lin.at(towr::kPos).transpose() << std::endl;

  std::cout << "EE initial positions: \n";
  for (int i = 0; i < formulation.model_.kinematic_model_->GetNumberOfEndeffectors(); ++i) {
    std::cout << "Position for the limb: " << i << " "
              << formulation.initial_ee_W_.at(i).transpose() << std::endl;
  }

  //must be called to override the constructor
  params.SetConstraints();

//  if (Parameters::robot_has_wheels_ == false)
//    params.SetSwingConstraint();

  // Pass this information to the actual solver
  ifopt::Problem nlp;
  SplineHolder solution;

  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);

  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);

  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("linear_solver", "ma57");
  solver->SetOption("ma57_pre_alloc", 10.0);
  solver->SetOption("max_cpu_time", 80.0);
  //solver->SetOption("jacobian_approximation", "finite-difference-values");

//  solver->SetOption("max_iter", 0);
//  solver->SetOption("derivative_test", "first-order");
//  solver->SetOption("print_level", 4);
//  solver->SetOption("derivative_test_perturbation", 1e-5);
//  solver->SetOption("derivative_test_tol", 1e-3);

  solver->Solve(nlp);

  printTrajectory(solution);

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6);  // t=0.0 throws ROS exception
  auto final_trajectory = GetTrajectory(solution);
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired,
                         formulation.terrain_.get());

  bag.close();

}
