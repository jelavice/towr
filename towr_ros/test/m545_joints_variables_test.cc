/*
 * m545_joints_variables_test.cc
 *
 *  Created on: Aug 31, 2018
 *      Author: jelavice
 */

#include <cmath>

#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <ifopt/ipopt_solver.h>

#include <std_msgs/Int32.h>

#include <towr/initialization/gait_generator.h>
#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr/nlp_formulation_extended.h>
#include <towr/terrain/examples/height_map_examples.h>

#include <towr/models/examples/m545_model_with_joints.h>


using namespace towr;

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

    t += 0.2;
  }
}

void setParameters(NlpFormulationExtended *formulation,
                   const std::string &urdfDescription)
{

  int n_ee = formulation->model_.kinematic_model_->GetNumberOfEndeffectors();
  auto model = formulation->model_.kinematic_model_->as<KinematicModelWithJoints>();

  //set initial position of the EE
  KinematicModelWithJoints::EEPos ee_pos = model->GetNominalStanceInBase();

  //set the z coordinate to be zero (simple transformation from base to world)
  for (auto &x : ee_pos)
    x.z() = 0.0;

  formulation->initial_ee_W_ = ee_pos;

  auto params = formulation->params_->as<ParametersExtended>();

  int boom_limb_id = static_cast<int>(loco_m545::RD::LimbEnum::BOOM);
  for (int i = 0; i < n_ee; ++i) {
    params->ee_phase_durations_.push_back( { 1.0 });
    if (i == boom_limb_id)
      params->ee_in_contact_at_start_.push_back(false);
    else
      params->ee_in_contact_at_start_.push_back(true);
  }

  params->use_bounds_initial_ee_pos.at(boom_limb_id) = false;

  double base_height = model->GetBasePositionFromFeetPostions().z();
  formulation->initial_base_.lin.at(towr::kPos) << 0.0, 0.0, base_height;
  formulation->final_base_.lin.at(towr::kPos) << 0.0, 0.0, base_height;

  //base stuff
  params->bounds_final_ang_pos = {};
  params->bounds_final_ang_vel = {X,Y,Z};
  params->bounds_final_lin_pos = {X,Y};
  params->bounds_final_lin_vel = {X,Y,Z};

  params->bounds_initial_ang_pos = {};
  params->bounds_initial_ang_vel = {X,Y,Z};
  params->bounds_initial_lin_pos = {};
  params->bounds_initial_lin_vel = {X,Y,Z};

  params->AddBaseVariables();
  params->AddEEMotionVariables();
  params->AddContactForceVariables();
  params->AddJointVariables();

  params->SetTerrainConstraint();
  params->SetDynamicConstraint();
  params->SetKinematicConstraint();
  params->SetForceConstraint();

  params->SetJointPolynomialDuration(0.02);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "m545_joints_test");
  ros::NodeHandle nh;

  //get the excavator model
  std::string urdfDescription;
  nh.getParam("romo_mm_description", urdfDescription);

  const double dt = 0.1;
  NlpFormulationExtended formulation;
  formulation.model_ = RobotModel(RobotModel::m545WithJoints, urdfDescription, dt);
  int n_ee = formulation.model_.kinematic_model_->GetNumberOfEndeffectors();
  formulation.params_ = nullptr;  // reset the default params
  formulation.params_ = std::make_shared<ParametersExtended>(n_ee);
  ParametersExtended *params = formulation.params_->as<ParametersExtended>();

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);


  setParameters(&formulation, urdfDescription);

  // Pass this information to the actual solver
  ifopt::Problem nlp;
  SplineHolderExtended solution;

  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
//
//  for (auto c : formulation.GetConstraints(solution))
//    nlp.AddConstraintSet(c);
//
//  for (auto c : formulation.GetCosts())
//    nlp.AddCostSet(c);
//
//  auto solver = std::make_shared<ifopt::IpoptSolver>();
//  solver->SetOption("linear_solver", "ma57");
//  solver->SetOption("ma57_pre_alloc", 10.0);
//  solver->SetOption("max_cpu_time", 80.0);
  //solver->SetOption("jacobian_approximation", "finite-difference-values");

//  solver->SetOption("max_iter", 0);
//  solver->SetOption("derivative_test", "first-order");
//  solver->SetOption("print_level", 4);
//  solver->SetOption("derivative_test_perturbation", 1e-5);
//  solver->SetOption("derivative_test_tol", 1e-3);

//  solver->Solve(nlp);
//
//  //printTrajectory(solution);
//  {
//    // Defaults to /home/user/.ros/
//    towr::M545TrajectoryManager trajectory_manager(formulation.terrain_.get());
//    std::string bag_file = "towr_trajectory.bag";
//    rosbag::Bag bag;
//    bag.open(bag_file, rosbag::bagmode::Write);
//    ::ros::Time t0(1e-6);  // t=0.0 throws ROS exception
//
//    trajectory_manager.SaveTrajectoryInRosbagCartesian(bag, xpp_msgs::robot_state_desired,
//                                                       solution);
//
//    bag.close();
//  }
//
//  {
//    towr::M545TrajectoryManager trajectory_manager(formulation.terrain_.get());
//    std::string prefix =
//        "/home/jelavice/Documents/catkin_workspaces/towr_ws/src/xpp/xpp_examples/bags";
//    std::string bag_file = prefix + "/m545.bag";
//    rosbag::Bag bag;
//    bag.open(bag_file, rosbag::bagmode::Write);
//    ::ros::Time t0(1e-6);  // t=0.0 throws ROS exception
//
//    trajectory_manager.SaveTrajectoryInRosbagJoints(bag, xpp_msgs::joint_desired, solution);
//
//    bag.close();
//  }

  std::cout << "Exititng" << std::endl;

  return 0;
}

