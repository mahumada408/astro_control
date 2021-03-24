#include <astro_control/convex_mpc/convex_mpc.h>
#include <ros/ros.h>

ConvexMpc::ConvexMpc(const int planning_horizon, const double timestep) : planning_horizon_(planning_horizon), timestep_(timestep) {
  plan_trajectory_.reserve(planning_horizon_);
}

void ConvexMpc::Compute() {
  // First we get the foot positions expressed in the world frame.
  // The foot position vectors are defined as the as the vector from the
  // body base frame origin B0 to the foot frame origin (e.g. FL0 for front left).
  const std::vector<Eigen::Vector3d> foot_positions = quadruped_.foot_positions();
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

void ConvexMpc::UpdateRobotPose(const std::vector<Eigen::Isometry3d>& foot_poses, const Eigen::Isometry3d& base_pose) {
  quadruped_.SetFootPositions(foot_poses);
  quadruped_.SetRobotPose(base_pose);
}