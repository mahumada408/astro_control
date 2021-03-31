#include <astro_control/floating_base/floating_base.h>

#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>


FloatingBase::FloatingBase() {
  FloatingBase(mass_, i_xx_, i_yy_, i_zz_);
}

FloatingBase::FloatingBase(double mass, double i_xx, double i_yy, double i_zz)
    : mass_(mass), i_xx_(i_xx), i_yy_(i_yy), i_zz_(i_zz) {
  // Set robot's body inertia.
  Eigen::Matrix<double, 3, 1> Id;
  Id << i_xx_, i_yy_, i_zz_;

  inertia_.setZero();
  inertia_.diagonal() = Id;
  std::cout << "inertia constructor" << std::endl;
  std::cout << inertia_ << std::endl;

  r_yaw_.setZero();
  A_continuous_.setZero();
  B_continuous_.setZero();
  robo_state_.setZero();
  robo_state_[State_idx::g] = -9.81;
}

void FloatingBase::SetFootPositions(const std::vector<Eigen::Isometry3d>& foot_poses) {

  foot_poses_.clear();
  foot_poses_ = foot_poses;
}

void FloatingBase::SetRobotPosition(const Eigen::Vector3d& robo_pos) {
  robo_state_[State_idx::x] = robo_pos.x();
  robo_state_[State_idx::y] = robo_pos.y();
  robo_state_[State_idx::z] = robo_pos.z();
}

void FloatingBase::SetOrientation(const Eigen::Matrix3d robo_rotation) {
  Eigen::Vector3d euler_angles = robo_rotation.eulerAngles(0, 1, 2);

  robo_state_[State_idx::roll] = euler_angles.x();
  robo_state_[State_idx::pitch] = euler_angles.y();
  robo_state_[State_idx::yaw] = euler_angles.z();
}

void FloatingBase::SetRobotVelocities(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity) {
  robo_state_[State_idx::x_dot] = linear_velocity.x();
  robo_state_[State_idx::y_dot] = linear_velocity.y();
  robo_state_[State_idx::z_dot] = linear_velocity.z();
  robo_state_[State_idx::roll_dot] = angular_velocity.x();
  robo_state_[State_idx::pitch_dot] = angular_velocity.y();
  robo_state_[State_idx::yaw_dot] = angular_velocity.z();
}

void FloatingBase::SetRobotPose(const Eigen::Isometry3d& robo_pose, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity) {
  SetRobotPosition(robo_pose.translation());
  SetOrientation(robo_pose.rotation());
  SetRobotVelocities(linear_velocity, angular_velocity);

  // Update rotation matrix with yaw angle.
  r_yaw_ << cos(robo_state_[State_idx::yaw]), -sin(robo_state_[State_idx::yaw]), 0,
            -sin(robo_state_[State_idx::yaw]), cos(robo_state_[State_idx::yaw]), 0,
            0, 0, 0;
}

Eigen::Matrix3d FloatingBase::SkewSymmetricFoot(const Eigen::Vector3d& foot_pos) {
  // Create a skew symmetric matrix from the foot position.
  // The skew matrix will have the following format:
  // 0, -z, y
  // z, 0, -x
  // -y, x, 0

  Eigen::Matrix3d skew_pos;
  skew_pos << 0, -foot_pos.z(), foot_pos.y(),
              foot_pos.z(), 0, -foot_pos.x(),
              -foot_pos.y(), foot_pos.x(), 0;
  std::cout << "skew pos" << std::endl;
  std::cout << skew_pos << std::endl;
  return skew_pos;
}

Eigen::Matrix3d FloatingBase::InertiaPos(const Eigen::Vector3d& foot_pos) {
  std::cout << "inertia" << std::endl;
  std::cout << inertia_ << std::endl;

  Eigen::Matrix<double, 3, 1> Id;
  Id << i_xx_, i_yy_, i_zz_;
  inertia_.diagonal() = Id;

  return inertia_.inverse() * SkewSymmetricFoot(foot_pos);
}

void FloatingBase::UpdateDynamics() {
  // Get inertia in the global frame.
  // Eigen::Matrix3d inertia_global{r_yaw_ * inertia_ * r_yaw_.transpose()};

  // Update the state space A matrix.
  // Zero it out first just in case.
  A_continuous_.setZero();
  A_continuous_.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();
  A_continuous_.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
  A_continuous_(8, 12) = 1.0; // gravity

  // Update the state space B matrix.
  // Zero it out first just in case.
  B_continuous_.setZero();

  for (int i = 0; i < Foot::foot_count; ++i) {
    B_continuous_.block(6, i * 3, 3, 3) = Eigen::Matrix3d::Identity() / mass_;
    B_continuous_.block(9, i * 3, 3, 3) = InertiaPos(foot_poses_[i].translation());
  }

  std::cout << "---------- A_continuous ----------" << std::endl;
  std::cout << A_continuous_ << std::endl;
  std::cout << "---------- B_continuous ----------" << std::endl;
  std::cout << B_continuous_ << std::endl;
}

//    ┌   ┐      ┌      ┐ ┌   ┐
//    │   │      │      │ │   │
//  d │ X │  ─── │ A  B │ │ X │
//  _ │   │  ─── │      │ │   │
// dt │ U │      │ 0  0 │ │ U │
//    │   │      │      │ │   │
//    └   ┘      └      ┘ └   ┘

void FloatingBase::DiscretizeDynamics() {
  // Combine A and B matrix into one super 25x25 matrix.
  const int matrix_size = State_idx::state_count + Control::control_count;
  Eigen::Matrix<double, matrix_size, matrix_size> AB_continuous;
  Eigen::Matrix<double, matrix_size, matrix_size> exponent_matrix;
  AB_continuous.setZero();
  exponent_matrix.setZero();
  AB_continuous.block(0, 0, State_idx::state_count, State_idx::state_count) = A_continuous_;
  AB_continuous.block(0, State_idx::state_count, State_idx::state_count, Control::control_count) = B_continuous_;
  AB_continuous = 0.030 * AB_continuous;  // 30 ms discretization.
  exponent_matrix = AB_continuous.exp();
  A_discrete_ = exponent_matrix.block(0, 0, State_idx::state_count, State_idx::state_count);
  B_discrete_ = exponent_matrix.block(0, State_idx::state_count, State_idx::state_count, Control::control_count);
}