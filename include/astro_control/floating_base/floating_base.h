#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "tf/transform_listener.h"

// Floating base model for a quadruped system. The model is defined as a floating base with leg enumeration as shown
// below. The forces applied to this model are only those imposed by gravity, and the reaction forces at the point
// of contact for each foot.
//
//        front
//  +-----------------+
//  |                 |
//  | 1             2 |
//  | FL_foot  FR_foot|
//  |       ^ Bx      |
//  |       |         |
//  |   By  |         |
//  |   <---+         |
//  |                 |
//  |                 |
//  |                 |
//  |                 |
//  | 3             4 |
//  | FL_foot  FR_foot|
//  +-----------------+

class FloatingBase {
 public:

  // States for the model and associates indices.
  enum State_idx {
    x = 0,
    y = 1, 
    z = 2, 
    x_dot = 3, 
    y_dot = 4, 
    z_dot = 5,
    roll = 6,
    pitch = 7,
    yaw = 8,
    roll_dot = 9,
    pitch_dot = 10,
    yaw_dot = 11,
    g = 12,
    state_count
  };

  struct State {
    double x = 0;
    double y = 0; 
    double z = 0;
    double x_dot = 0;
    double y_dot = 0;
    double z_dot = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double roll_dot = 0;
    double pitch_dot = 0;
    double yaw_dot = 0;
    double g = 0;
  };

  // Control indices for floating base model.
  enum Control {
    fx_fl = 0,
    fy_fl = 1,
    fz_fl = 2,
    fx_fr = 3,
    fy_fr = 4,
    fz_fr = 5,
    fx_rl = 6,
    fy_rl = 7,
    fz_rl = 8,
    fx_rr = 9,
    fy_rr = 10,
    fz_rr = 11,
    control_count
  };

  // Indices for the individual feet.
  enum Foot {
    fl = 0,
    fr = 1,
    rl = 2, 
    rr = 3,
    foot_count
  };

  FloatingBase();
  FloatingBase(double mass, double Ixx, double Iyy, double Izz);
  ~FloatingBase() {}

  void SetFootPositions(const std::vector<Eigen::Isometry3d>& foot_poses);

  // Sets the pose of the robots base frame (at the CG) in the world frame.
  void SetRobotPose(const Eigen::Isometry3d& robo_pose, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity);

  // Update the continuous linear dynamics.
  // x_dot = Ax + Bu
  void UpdateDynamics();

  void DiscretizeDynamics();

  Eigen::Matrix<double, 13, 13> A() { return A_continuous_; }

  Eigen::Matrix<double, 13, 12> B() { return B_continuous_; }

  Eigen::Matrix<double, 13, 13> A_dt() { return A_discrete_; }

  Eigen::Matrix<double, 13, 12> B_dt() { return B_discrete_; }

  std::vector<Eigen::Isometry3d> foot_poses() { return foot_poses_; }

  Eigen::Matrix<double, State_idx::state_count, 1> robot_state() { return robo_state_; }

  double mu() { return mu_; }

  double mass() { return mass_; }

 private:
  // Form skew symmetric matrix for foot position.
  Eigen::Matrix3d SkewSymmetricFoot(const Eigen::Vector3d& foot_pos);

  // Matrix multiplication between inertia tensor and the position vector
  // from the CG to the foot location.
  Eigen::Matrix3d InertiaPos(const Eigen::Vector3d& foot_pos);

  // Sets the position of the origin of the body frame in the world frame, expressed in the world frame.
  void SetRobotPosition(const Eigen::Vector3d& robo_pos);

  // Sets the orientation of the body frame in the world frame, expressed in the world frame.
  void SetOrientation(const Eigen::Matrix3d robo_quat);

  void SetRobotVelocities(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity);

  // Mass of the robot.
  // Default to A1.
  double mass_ = 12.454;

  // Robot inertia tensor in the quadruped body frame.
  // Default to A1.
  double i_xx_ = 0.01683993;
  double i_yy_ = 0.056579028;
  double i_zz_ = 0.064713601;
  Eigen::Matrix3d inertia_;

  // Friction.
  double mu_ = 0.5;

  // Rotation matrix about the z axis (yaw).
  Eigen::Matrix3d r_yaw_;

  // Position of foot from the body cm expressed in the world frame.
  Eigen::Vector3d foot_fl_;
  Eigen::Vector3d foot_fr_;
  Eigen::Vector3d foot_rl_;
  Eigen::Vector3d foot_rr_;
  std::vector<Eigen::Isometry3d> foot_poses_;

  Eigen::Matrix<double, 13, 13> A_continuous_;
  Eigen::Matrix<double, 13, 12> B_continuous_;
  Eigen::Matrix<double, 13, 13> A_discrete_;
  Eigen::Matrix<double, 13, 12> B_discrete_;

  // State of the robot. Note that all state variables must be expressed in the world frame.
  Eigen::Matrix<double, State_idx::state_count, 1> robo_state_;
};