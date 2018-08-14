/*
 * jacobians_unit_test.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: jelavice
 */

#include <towr/terrain/examples/height_map_examples.h>

#include <cmath>

#include <string>

#include <ros/ros.h>

#include <towr/models/examples/m545_model.h>

using namespace towr;

const double perturbation = 1e-4;

Eigen::MatrixXd getJacobianNumerically(M545KinematicModelFull* kinematic_model, int limbId,
                                       Eigen::VectorXd joint_angles)
{

  int num_joints = joint_angles.size();

  Eigen::VectorXd unit_vector(num_joints);

  Eigen::MatrixXd jac;
  jac.resize(3, num_joints);

  for (int j = 0; j < num_joints; ++j) {
    Eigen::Vector3d left, right;
    unit_vector.setZero();
    unit_vector(j) = 1.0;

    kinematic_model->UpdateModel(joint_angles + perturbation * unit_vector, limbId);
    right = kinematic_model->GetEEPositionsBase(limbId);
    //std::cout << "right: " << right.transpose() << std::endl;
    //std::cout << "right angles: " << (joint_angles + perturbation * unit_vector).transpose() << std::endl;
    //kinematic_model->printCurrentJointPositions();

    kinematic_model->UpdateModel(joint_angles - perturbation * unit_vector, limbId);
    left = kinematic_model->GetEEPositionsBase(limbId);
    //std::cout << "Left: " << left.transpose() << std::endl;
    //std::cout << "left angles: " << (joint_angles - perturbation * unit_vector).transpose() << std::endl;
    //kinematic_model->printCurrentJointPositions();

    jac.col(j) = (right - left) / (2 * perturbation);

  }

  return jac;

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "m545_example_node");
  ros::NodeHandle nh;

  //get the excavator model
  std::string urdfDescription;
  nh.getParam("romo_mm_description", urdfDescription);

  // Kinematic limits and dynamic parameters of the hopper
  constexpr double dt = 0.01;

  auto kinematic_model = std::make_shared<M545KinematicModelFull>(urdfDescription, dt);

  std::vector<Eigen::MatrixXd> ee_jac_base_num;
  M545KinematicModelFull::EEJac ee_jac_base;
  std::vector<Eigen::VectorXd> joint_angles_vector;

  ee_jac_base_num.resize(kinematic_model->numEE);
  ee_jac_base.resize(kinematic_model->numEE);
  joint_angles_vector.resize(kinematic_model->numEE);

  for (int i = 0; i < kinematic_model->numEE; ++i) {
    ee_jac_base.at(i).resize(3, kinematic_model->GetNumDof(i));
    ee_jac_base_num.at(i).resize(3, kinematic_model->GetNumDof(i));
    joint_angles_vector.at(i).resize(kinematic_model->GetNumDof(i));
  }

  const int numTests = 10000;
  int num_failed = 0;
  for (int n = 0; n < numTests; ++n) {

    //iterate over the limbs
    bool failed = false;
    for (int i = 0; i < kinematic_model->numEE; ++i) {

      joint_angles_vector.at(i).setRandom() * 10;
      kinematic_model->UpdateModel(joint_angles_vector.at(i), i);
      ee_jac_base.at(i) = kinematic_model->GetTranslationalJacobiansWRTjointsBase(i);

      ee_jac_base_num.at(i) = getJacobianNumerically(kinematic_model.get(), i,
                                                     joint_angles_vector.at(i));

      Eigen::MatrixXd res = ee_jac_base.at(i) - ee_jac_base_num.at(i);
      if (res.lpNorm<Eigen::Infinity>() > 1e-3) {
        std::cout << "Big discrepancy in the jacobian for the limb: " << i << std::endl;
        std::cout << "Analytical: \n" << ee_jac_base.at(i) << std::endl;
        std::cout << "Numerical: \n" << ee_jac_base_num.at(i) << std::endl;
        std::cout << "Joints: " << joint_angles_vector.at(i).transpose() << std::endl;
        failed = true;
      }
    }

    if (failed) {
      ++num_failed;
    }

  }

  std::cout << "Tests passed: " << (numTests - num_failed) << "/" << numTests << std::endl;

  return 0;

}
