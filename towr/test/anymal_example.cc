/*
 * anymal_example.cc
 *
 *  Created on: Jul 23, 2018
 *      Author: jelavice
 */

/******************************************************************************
 Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

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
  RobotModel model(RobotModel::Anymal);

  // set the initial position of the hopper
  BaseState initial_base;
  initial_base.lin.at(kPos).z() = 0.42;

  const double x_nominal_b = 0.34;
  const double y_nominal_b = 0.19;
  const double z_nominal_b = 0.0;

  towr::TOWR::FeetPos nominal_stance(4);

  nominal_stance.at(towr::QuadrupedIDs::LF) << x_nominal_b, y_nominal_b, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::RF) << x_nominal_b, -y_nominal_b, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::LH) << -x_nominal_b, y_nominal_b, z_nominal_b;
  nominal_stance.at(towr::QuadrupedIDs::RH) << -x_nominal_b, -y_nominal_b, z_nominal_b;

  // define the desired goal state of the hopper
  BaseState goal;
  goal.lin.at(towr::kPos) << 0.0, 0.0, 0.42;

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
