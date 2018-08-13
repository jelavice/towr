/*
 * range_of_motion_constraint_complicated.cc
 *
 *  Created on: Aug 2, 2018
 *      Author: jelavice
 */

#include <towr/constraints/range_of_motion_constraint_joints.h>
#include <towr/variables/variable_names.h>

namespace towr {

RangeOfMotionConstraintJoints::RangeOfMotionConstraintJoints(KinematicModelJoints::Ptr model,
                                                             double T, double dt, EE ee,
                                                             const SplineHolder& spline_holder)
    : TimeDiscretizationConstraint(T, dt, "rangeofmotionjoints-" + std::to_string(ee))
{
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);

  ee_ = ee;

  ee_motion_ = spline_holder.ee_motion_.at(ee_);
  joints_motion_ = spline_holder.joint_motion_.at(ee_);

  if (joints_motion_ == nullptr)
    throw std::runtime_error("Nullptr is the joint motion splines");

  kinematic_model_ = model;

  //todo make this work for all terrain (also inclinations), gonna need more constraints

  //need to include the constraints for all the joint bounds as well
  num_constraints_per_node_ = kinematic_model_->GetNumDof(ee_);

  num_constraints_per_node_ += k3D + 1;  // position (3 position constraints, and a yaw angle of the wheel)

  //cache these guys
  lower_bounds_ = kinematic_model_->GetLowerJointLimits(ee_);
  upper_bounds_ = kinematic_model_->GetUpperJointLimits(ee_);

  SetRows(GetNumberOfNodes() * num_constraints_per_node_);
}

//this one is called first it seems
void RangeOfMotionConstraintJoints::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{

  int dimension = 0;

  //std::cout << "Dimension of the g" << g.rows() << std::endl;

  /* first get all the variables*/
  //todo make sure that the caller has set the right size of the state
  VectorXd joint_positions(kinematic_model_->GetNumDof(ee_));

  joint_positions = joints_motion_->GetPoint(t).p();

  // now update the value of the ee constraint
  // first get the position and euler shit
  Vector3d base_position = base_linear_->GetPoint(t).p();
  Vector3d base_angular = base_angular_.GetEulerAngles(t);

  // second update the model
  kinematic_model_->UpdateModel(joint_positions, base_angular, base_position);

  //then get all the positions and shit that I need
  Vector3d ee_position_from_joints = kinematic_model_->GetEEPositionsWorld().at(ee_);
  Vector3d ee_position = ee_motion_->GetPoint(t).p();

  //take out the yaw
  double wheel_yaw = kinematic_model_->GetEEOrientation().at(ee_).z();

  //put it in the constraint
  Eigen::Vector2d ee_vel = ee_motion_->GetPoint(t).v().segment(0, 2);  // get the x and y velocity

  /*now update the actual constraints*/

  //joint range constraint
  g.middleRows(GetRow(k, dimension), joint_positions.size()) = joint_positions;
  dimension += joint_positions.size();

  //endeffector position
  g.middleRows(GetRow(k, dimension), dim3) = ee_position_from_joints - ee_position;
  dimension += dim3;

  //no slip in lateral direction
  g(GetRow(k, dimension)) = ee_vel.x() * std::sin(wheel_yaw) - ee_vel.y() * std::cos(wheel_yaw);  // last row anyway
}
void RangeOfMotionConstraintJoints::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{

  int dimension = 0;

  //first work out the joint bounds
  for (int j = 0; j < kinematic_model_->GetNumDof(ee_); ++j) {
    ifopt::Bounds b;
    b.lower_ = lower_bounds_(j);
    b.upper_ = upper_bounds_(j);
    bounds.at(GetRow(k, dimension++)) = b;
  }

  //now workout the endeffector constraint bounds
  //equality constarints
  for (int dim = 0; dim < dim3; ++dim) {
    bounds.at(GetRow(k, dimension++)) = ifopt::BoundZero;
  }

  //then workout the heading direction
  //equality constraint
  bounds.at(GetRow(k, dimension)) = ifopt::BoundZero;

}

void RangeOfMotionConstraintJoints::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                                             Jacobian& jac) const
{

  // need

  // 1. joints here I should also fit the end yaw angle constraint

  // 2. base lin

  // 3. base ang

  // 4. end effector position and end effectors felocity

  //start with the joints since it is the easiest

  int row_start = GetRow(k, 0);

  //std::cerr << "k: " << k << std::endl;
//  if (k == 0)
//    std::cerr << "Size of the jacobian for " << var_set << " : \n" << jac.rows() << "x"
//              << jac.cols() << std::endl;


  if (var_set == id::JointNodes(ee_)) {

    // jacobian wrt to joints
    int dim = kinematic_model_->GetNumDof(ee_);
    jac.middleRows(row_start, dim) = joints_motion_->GetJacobianWrtNodes(t, kPos);
    row_start += dim;

    //now work out the end-effector constarint
    dim = dim3;

//    auto x = kinematic_model_->GetTranslationalJacobiansWRTjoints().at(ee_);
//    std::cout << "translational jacobian size: "<< x.rows() << "x" << x.cols() << std::endl;
//    std::cout << kinematic_model_->GetTranslationalJacobiansWRTjoints().at(ee_) << std::endl;
//
//    auto y = joints_motion_->GetJacobianWrtNodes(t, kPos);
//    std::cout << "jacobian wrt to nodes: " << y.rows() << "x" << y.cols() << std::endl;
//    std::cout << std::endl << joints_motion_->GetJacobianWrtNodes(t, kPos) << std::endl;

    Jacobian temp = kinematic_model_->GetTranslationalJacobiansWRTjoints().at(ee_)
            * joints_motion_->GetJacobianWrtNodes(t, kPos);
    jac.middleRows(row_start, dim) = temp;
    row_start += dim;

    //need velocity and angle for the last constraint
    //dis is the wheel heading constraint
    Eigen::Vector2d ee_vel = ee_motion_->GetPoint(t).v().segment(0, 2);  // get the x and y velocity
    double yaw = kinematic_model_->GetEEOrientation().at(ee_).z();
    temp = kinematic_model_->GetOrientationJacobiansWRTjoints().at(ee_)
            .row(2) * (ee_vel.x() * std::cos(yaw) + ee_vel.y() * std::sin(yaw));

    jac.row(row_start) = temp;

  }

  if (var_set == id::base_lin_nodes) {
    //skip the first constraint since the derivative is zero wrt to base lin nodes

    //work out the end-effector constraint
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint
    int dim = dim3;
    Jacobian temp = kinematic_model_->GetTranslationalJacobianWRTbasePosition().at(
        ee_) * base_linear_->GetJacobianWrtNodes(t, kPos);
    jac.middleRows(row_start, dim) = temp;
    row_start += dim;

    // jacobian of the last constraint is also zero wrt to the base lin nodes

  }

  if (var_set == id::base_ang_nodes) {
    //skip the first constraint since the derivative is zero wrt to base ang nodes

    //work out the end-effector constraint
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint
    int dim = dim3;
    Jacobian temp = kinematic_model_->GetTranslatinalJacobianWRTbaseOrientation()
            .at(ee_) * base_angular_.GetNodeSpline()->GetJacobianWrtNodes(t, kPos);
    jac.middleRows(row_start, dim) = temp;
    row_start += dim;

    // jacobian of the last constraint is also zero wrt to the base lin nodes
    dim = 1;
    Eigen::Vector2d ee_vel = ee_motion_->GetPoint(t).v().segment(0, 2);  // get the x and y velocity
    double yaw = kinematic_model_->GetEEOrientation().at(ee_).z();

//    std::cerr << "Size of the jacobian: \n" << jac.rows() << "x" << jac.cols() << std::endl;
//    std::cerr << "row start \n" << row_start << std::endl;
//    std::cerr << "the one from the kinematic model: \n"
//              << kinematic_model_->GetOrientationJacobiansWRTbaseOrientation().at(ee_) << std::endl;
//
//    auto angJac = base_angular_.GetNodeSpline()->GetJacobianWrtNodes(t, kPos);
//    std::cerr << "ang spline \n" << base_angular_.GetNodeSpline()->GetJacobianWrtNodes(t, kPos)
//              << std::endl;
//    std::cerr << "Size the one in the ang spline" << angJac.rows() << "x" << angJac.cols()
//              << std::endl;

    temp = kinematic_model_->GetOrientationJacobiansWRTbaseOrientation().at(ee_)
        * base_angular_.GetNodeSpline()->GetJacobianWrtNodes(t, kPos) * (ee_vel.x() * std::cos(yaw) + ee_vel.y() * std::sin(yaw));

//    std::cout << "Size of the temp: " << temp.rows() << "x" << temp.cols() << std::endl;
//    std::cout << "Velocity: " << ee_vel.transpose() << std::endl;

    jac.row(row_start) = temp.row(2); //last row anyway

    //std::cerr << "================" << std::endl << std::endl;

  }


  if (var_set == id::EEMotionNodes(ee_)) {

    //skip the first constraint since the derivative is zero wrt to base ang nodes

    //work out the end-effector constraint
    row_start += kinematic_model_->GetNumDof(ee_);  //need to skip the rows corresponding to the first constraint
    int dim = dim3;
    Jacobian temp = (-ee_motion_->GetJacobianWrtNodes(t, kPos)).eval();
    jac.middleRows(row_start, dim) = temp;
    row_start += dim;

    //now work out the wheel direction constraint
    dim = 1;
    double yaw = kinematic_model_->GetEEOrientation().at(ee_).z();
    Eigen::Vector3d constraint_partial_derivative(std::cos(yaw), std::sin(yaw), 0.0);
    temp = (constraint_partial_derivative
        * ee_motion_->GetJacobianWrtNodes(t, kPos)).eval().sparseView();
    jac.middleRows(row_start, dim) = temp;

  }

}

int RangeOfMotionConstraintJoints::GetRow(int node, int dimension) const
{
  return node * num_constraints_per_node_ + dimension;

}

} /*namespace*/
