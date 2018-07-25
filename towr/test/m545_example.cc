/*
 * m545_example.cc
 *
 *  Created on: Jul 17, 2018
 *      Author: jelavice
 */

#include <cmath>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/models/robot_model.h>
#include <towr/towr.h>
#include <towr/models/endeffector_mappings.h>

#include <ifopt/ipopt_solver.h>

using namespace towr;

int main()
{
  // terrain
  auto terrain = std::make_shared<FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the hopper
  RobotModel model(RobotModel::m545);
  Parameters::robot_has_wheels_ = true;

  // set the initial position of the hopper
  BaseState initial_base;
  initial_base.lin.at(kPos).z() = 0.95;

  const double x_nominal_b_front = 2.7;
  const double y_nominal_b_front = 1.6;

  const double x_nominal_b_hind = 2.05;
  const double y_nominal_b_hind = 1.26;

  const double z_nominal_b = 0.0;



  towr::TOWR::FeetPos nominal_stance(4);

  nominal_stance.at(towr::QuadrupedIDs::LF) << x_nominal_b_front,   y_nominal_b_front, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::RF) << x_nominal_b_front,  -y_nominal_b_front, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::LH) << -x_nominal_b_hind,   y_nominal_b_hind, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::RH) << -x_nominal_b_hind,  -y_nominal_b_hind, z_nominal_b;

  // define the desired goal state of the hopper
  BaseState goal;
  goal.lin.at(towr::kPos) << 0.0, 0.0, 0.95;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  Parameters params;
  // here we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____

  for (int i =0; i < 4; ++i) {
    params.ee_phase_durations_.push_back( { 1.0 });
    params.ee_in_contact_at_start_.push_back(true);
  }
  params.SetSwingConstraint();

  // Pass this information to the actual solver
  TOWR towr;
  towr.SetInitialState(initial_base, nominal_stance);
  towr.SetParameters(goal, params, model, terrain);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  //solver->SetOption("jacobian_approximation", "finite-difference-values");
  towr.SolveNLP(solver);

  auto x = towr.GetSolution();

  // Print out the trajecetory at discrete time samples
  using namespace std;
  cout.precision(2);
  cout << fixed;
  cout << "\n====================\n anymal trajectory: \n====================\n";

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

    cout << "Contact force x,y,z:          \n";
    cout << "LF: " << x.ee_force_.at(LF)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "RF: " << x.ee_force_.at(RF)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "LH: " << x.ee_force_.at(LH)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "RH: " << x.ee_force_.at(RH)->GetPoint(t).p().transpose() << "\t[N]" << endl;

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
