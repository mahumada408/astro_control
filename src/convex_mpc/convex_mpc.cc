#include <astro_control/convex_mpc/convex_mpc.h>

#include <ros/ros.h>

#include <unsupported/Eigen/MatrixFunctions>

using fsidx = FloatingBase::State_idx;
using fc = FloatingBase::Control;

ConvexMpc::ConvexMpc(const int planning_horizon, const double timestep) : planning_horizon_(planning_horizon), timestep_(timestep) {
  plan_trajectory_.reserve(planning_horizon_);
  H_.resize(3 * FloatingBase::Foot::foot_count * planning_horizon_, 3 * FloatingBase::Foot::foot_count * planning_horizon_);
  g_.resize(3 * FloatingBase::Foot::foot_count * planning_horizon_, 1);
  C_.resize(5 * FloatingBase::Foot::foot_count * planning_horizon_, 3 * FloatingBase::Foot::foot_count * planning_horizon_);

  H_.setZero();
  g_.setZero();
  C_.setZero();
}

void ConvexMpc::Compute() {
  // First we get the foot positions expressed in the world frame.
  // The foot position vectors are defined as the as the vector from the
  // body base frame origin B0 to the foot frame origin (e.g. FL0 for front left).

  //
  quadruped_.UpdateDynamics();
  quadruped_.DiscretizeDynamics();
  CondensedFormulation();

  // Get cost matricies.

  // convert to qpoases matricies

}

void ConvexMpc::GenerateTrajectory(const Eigen::Vector3d desired_velocity, const Eigen::Vector3d desired_angular_velocity) {
  // Yaw rate up to 200 deg/s (3.49 rad/s)
  // velocity from 0.5 to 1.5 m/s
  // desired_com_position = {0.0, 0.0, 0.42};

  // Get desired states from keyboard input.
  FloatingBase::State desired_state;
  for (int i = 0; i < planning_horizon_; ++i) {
    // Project the velocity to get desired position.
    // Fill up a state.
    desired_state.x = timestep_ * (i + 1) * desired_velocity.x();
    desired_state.y = timestep_ * (i + 1) * desired_velocity.y();
    desired_state.z = timestep_ * (i + 1) * desired_velocity.z();
    desired_state.x_dot = desired_velocity.x();
    desired_state.y_dot = desired_velocity.y();
    desired_state.z_dot = desired_velocity.z(); // Should be zero to stabilize the height.

    // Desired roll pitch yaw should be 0 for stabilization.
    desired_state.roll = 0.0;
    desired_state.pitch = 0.0;
    desired_state.yaw = 0.0;
    // Desired roll and pitch rate should be zero for stabilization.
    desired_state.roll_dot = desired_angular_velocity.x();
    desired_state.pitch_dot = desired_angular_velocity.y();
    desired_state.yaw_dot = desired_angular_velocity.z();

    desired_state.g = -9.81; // m/s^2

    plan_trajectory_.push_back(desired_state);
  }

}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ConvexMpc::StateRef() {
  Eigen::VectorXd state_ref;
  state_ref.resize(fsidx::state_count * planning_horizon_);

  for (size_t i = 0; i < planning_horizon_; ++i) {
    state_ref[fsidx::x * i] = plan_trajectory_[i].x;
    state_ref[fsidx::y * i] = plan_trajectory_[i].y;
    state_ref[fsidx::z * i] = plan_trajectory_[i].z;
    state_ref[fsidx::x_dot * i] = plan_trajectory_[i].x_dot;
    state_ref[fsidx::y_dot * i] = plan_trajectory_[i].y_dot;
    state_ref[fsidx::z_dot * i] = plan_trajectory_[i].z_dot;
    state_ref[fsidx::roll * i] = plan_trajectory_[i].roll;
    state_ref[fsidx::pitch * i] = plan_trajectory_[i].pitch;
    state_ref[fsidx::yaw * i] = plan_trajectory_[i].yaw;
    state_ref[fsidx::roll_dot * i] = plan_trajectory_[i].roll_dot;
    state_ref[fsidx::pitch_dot * i] = plan_trajectory_[i].pitch_dot;
    state_ref[fsidx::yaw_dot * i] = plan_trajectory_[i].yaw_dot;
    state_ref[fsidx::g * i] = plan_trajectory_[i].g;
  }

  return state_ref;
}

void ConvexMpc::UpdateRobotPose(const std::vector<Eigen::Isometry3d>& foot_poses, const Eigen::Isometry3d& base_pose, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity) {
  quadruped_.SetFootPositions(foot_poses);
  quadruped_.SetRobotPose(base_pose, linear_velocity, angular_velocity);
}

void ConvexMpc::CondensedFormulation() {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_qp;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_qp;

  A_qp.resize(fsidx::state_count * planning_horizon_, fsidx::state_count);
  B_qp.resize(fsidx::state_count * planning_horizon_, fc::control_count * planning_horizon_);

  A_qp.setZero();
  B_qp.setZero();

  // Set A
  // A_qp.block(0, 0, fsidx::state_count, fsidx::state_count) = Eigen::Matrix<double, fsidx::state_count, fsidx::state_count>::Identity();

  std::cout << "---------- A_dt ----------" << std::endl;
  std::cout << quadruped_.A_dt() << std::endl;
  for (int i = 0; i < planning_horizon_; ++i) {
    Eigen::Matrix<double, 13, 13> A_power;
    A_power = quadruped_.A_dt();
    for (int pow = 0; pow < i; ++pow) {
      A_power = A_power * quadruped_.A_dt();
    }
    A_qp.block(i * fsidx::state_count, 0, fsidx::state_count, fsidx::state_count) = A_power;

  }

  // Set B
  for (int i = 0; i < planning_horizon_; ++i) {
    // Go across the columns.
    for (int j = 0; j < planning_horizon_; ++j) {
      // Go across the rows.
      B_qp.block(j * fsidx::state_count, i * fc::control_count, fsidx::state_count, fc::control_count) = quadruped_.A_dt().pow(j - 1) * quadruped_.B_dt();
    }
  }

  std::cout << "---------- A_qp ----------" << std::endl;
  std::cout << A_qp.rows() << "x" << A_qp.cols() << std::endl;
  std::cout << A_qp << std::endl;
  std::cout << "---------- B_qp ----------" << std::endl;
  std::cout << B_qp.rows() << "x" << B_qp.cols() << std::endl;
  std::cout << B_qp << std::endl;

  // L is a diagonal matrix of weights for the state deviations. L has dimensions 13k x 13k.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
  
  // K is a diagonal matrix of weights for force magnitude. K has dimensions of 3nk x 3nk.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;

  L.resize(FloatingBase::State_idx::state_count * planning_horizon_, FloatingBase::State_idx::state_count * planning_horizon_);
  K.resize(3 * FloatingBase::Foot::foot_count * planning_horizon_, 3 * FloatingBase::Foot::foot_count * planning_horizon_);

  L.setZero();
  K.setZero();

  double alpha = 0.5;
  Eigen::VectorXd weights;
  weights.resize(fsidx::state_count);
  weights << 5, 5, 0.2, 0, 0, 10, 0, 0, 1, 1, 1, 0, 0;
  L.diagonal() = weights.replicate(planning_horizon_, 1);
  K.diagonal() = K.diagonal().setOnes() * alpha;

  std::cout << "---------- K ----------" << std::endl;
  std::cout << K << std::endl;
  std::cout << "---------- L ----------" << std::endl;
  std::cout << L << std::endl;

  H_ = 2 * (B_qp.transpose() * L * B_qp + K);
  g_ = 2 * B_qp.transpose() * L * (A_qp * quadruped_.robot_pose() - StateRef());

  // Constraint matrix;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> constraints;
  constraints.resize(5, 3);
  constraints << -1, 0, quadruped_.mu(),
                  1, 0, quadruped_.mu(),
                  0, -1, quadruped_.mu(),
                  0, 1, quadruped_.mu(),
                  0, 0, 1;
  
  for (int i = 0; i < planning_horizon_ * FloatingBase::Foot::foot_count; ++i) {
    C_.block(i * 5, i * 3, 5, 3) = constraints;
  }

  std::cout << "---------- C ----------" << std::endl;
  std::cout << C_ << std::endl;
}