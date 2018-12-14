/*
 * m545_model_with_joints_jacobians_test.cc
 *
 *  Created on: Aug 30, 2018
 *      Author: jelavice
 */

#include <towr/terrain/examples/height_map_examples.h>

#include <cmath>

#include <string>

#include <ros/ros.h>

#include <towr/models/examples/m545_model_with_joints.h>
#include <functional>

using namespace towr;

const double perturbation = 1e-4;

Eigen::MatrixXd getJacobianNumerically(M545KinematicModelWithJoints* kinematic_model, int limbId,
                                       Eigen::VectorXd joint_angles,
                                       std::function<Eigen::Vector3d(int)> func_to_evaluate)
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

    right = func_to_evaluate(limbId);
    //right = kinematic_model->GetEEPositionsBase(limbId);
    //std::cout << "right: " << right.transpose() << std::endl;
    //std::cout << "right angles: " << (joint_angles + perturbation * unit_vector).transpose() << std::endl;
    //kinematic_model->printCurrentJointPositions();

    kinematic_model->UpdateModel(joint_angles - perturbation * unit_vector, limbId);
    left = func_to_evaluate(limbId);
    //left = kinematic_model->GetEEPositionsBase(limbId);
    //std::cout << "Left: " << left.transpose() << std::endl;
    //std::cout << "left angles: " << (joint_angles - perturbation * unit_vector).transpose() << std::endl;
    //kinematic_model->printCurrentJointPositions();

    jac.col(j) = (right - left) / (2 * perturbation);

  }

  return jac;

}

void runTest(const ros::NodeHandle &nh, const std::string &which_test, int numTests)
{

  //get the excavator model
  std::string urdfDescription;
  nh.getParam("romo_mm_description", urdfDescription);

  constexpr double dt = 0.01;
  auto kinematic_model = std::make_shared<M545KinematicModelWithJoints>(urdfDescription, dt);

  std::vector<Eigen::MatrixXd> ee_jac_base_num;
  M545KinematicModelWithJoints::EEJac ee_jac_base;
  std::vector<Eigen::VectorXd> joint_angles_vector;

  ee_jac_base_num.resize(kinematic_model->numEE);
  ee_jac_base.resize(kinematic_model->numEE);
  joint_angles_vector.resize(kinematic_model->numEE);

  for (int i = 0; i < kinematic_model->numEE; ++i) {
    ee_jac_base.at(i).resize(3, kinematic_model->GetNumDof(i));
    ee_jac_base_num.at(i).resize(3, kinematic_model->GetNumDof(i));
    joint_angles_vector.at(i).resize(kinematic_model->GetNumDof(i));
  }

  int num_failed = 0;
  for (int n = 0; n < numTests; ++n) {

    //iterate over the limbs
    bool failed = false;
    for (int i = 0; i < kinematic_model->numEE; ++i) {

      joint_angles_vector.at(i).setRandom() * 10;
      kinematic_model->UpdateModel(joint_angles_vector.at(i), i);

      std::function<Eigen::Vector3d(int)> func_to_evaluate;

      if (which_test == "trans_test") {
        func_to_evaluate = [=](int limbId) {
          return kinematic_model->GetEEPositionBase(limbId);
        };

        ee_jac_base.at(i) = kinematic_model->GetTranslationalJacobiansWRTjointsBase(i);

      }

      if (which_test == "rot_test") {
        func_to_evaluate = [=](int limbId) {
          return kinematic_model->GetEEOrientationBase(limbId);
        };

        ee_jac_base.at(i) = kinematic_model->GetOrientationJacobiansWRTjointsBase(i);

      }

      if (which_test == "wheel_axis_test") {

        if (kinematic_model->EEhasWheel(i) == false)
          continue;

        func_to_evaluate = [=](int ee_id) {
          return kinematic_model->GetWheelAxisBase(ee_id);
        };

        ee_jac_base.at(i) = kinematic_model->GetWheelAxisJacobianBase(i);

      }

      if (which_test == "rotation_matrix_test") {

        Eigen::Vector3d random_vector;
        random_vector.Random();

        func_to_evaluate = [=](int ee_id) {
          return kinematic_model->GetRotationBaseToWheel(ee_id) * random_vector;
        };

        ee_jac_base.at(i) = kinematic_model->GetDerivOfRotVecMult(random_vector, i, false);

      }

      if (which_test == "rotation_matrix_inverse_test") {

        Eigen::Vector3d random_vector;
        random_vector.Random();

        func_to_evaluate = [=](int ee_id) {
          return kinematic_model->GetRotationBaseToWheel(ee_id).transpose() * random_vector;
        };

        ee_jac_base.at(i) = kinematic_model->GetDerivOfRotVecMult(random_vector, i, true);

      }

      ee_jac_base_num.at(i) = getJacobianNumerically(kinematic_model.get(), i,
                                                     joint_angles_vector.at(i), func_to_evaluate);

      Eigen::MatrixXd res = ee_jac_base.at(i) - ee_jac_base_num.at(i);
      double max_discrepancy = res.lpNorm<Eigen::Infinity>();
      if (max_discrepancy > 1e-3) {
        std::cout << "Big discrepancy in the jacobian for the limb: " << i << std::endl;
        std::cout << "Analytical: \n" << ee_jac_base.at(i) << std::endl;
        std::cout << "Numerical: \n" << ee_jac_base_num.at(i) << std::endl;
        std::cout << "Joints: " << joint_angles_vector.at(i).transpose() << std::endl;
        std::cout << "position/Orientation: " << func_to_evaluate(i).transpose() << std::endl;
        std::cout << "\n Max discrepancy: " << max_discrepancy << std::endl << std::endl;
        failed = true;
      }
    }

    if (failed) {
      ++num_failed;
    }

  }

  std::cout << "\n=================================================== \n";

  std::cout << "Test: " << which_test <<", tests passed: " << (numTests - num_failed) << "/"
              << numTests << std::endl;

  std::cout << "=================================================== \n";

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "m545_example_node");
  //seed std library random generator
  srand((unsigned int) time(nullptr));
  ros::NodeHandle nh;

  runTest(nh, "rotation_matrix_test", 10000);
  runTest(nh, "rotation_matrix_inverse_test", 10000);
  runTest(nh, "wheel_axis_test", 10000);
  runTest(nh, "trans_test", 10000);
  runTest(nh, "rot_test", 10000);

  return 0;

}
