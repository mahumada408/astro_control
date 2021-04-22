#include <astro_control/convex_mpc/convex_mpc.h>

#include <ros/ros.h>

#include <unsupported/Eigen/MatrixFunctions>
#include "astro_control/qpOASES/include/qpOASES/QProblem.hpp"


using fsidx = FloatingBase::State_idx;
using fc = FloatingBase::Control;

ConvexMpc::ConvexMpc(const int planning_horizon, const double timestep) : planning_horizon_(planning_horizon), timestep_(timestep) {
  plan_trajectory_.resize(planning_horizon_);
  FloatingBase quad;
  quadruped_ = quad;

  H_.resize(3 * FloatingBase::Foot::foot_count * planning_horizon_, 3 * FloatingBase::Foot::foot_count * planning_horizon_);
  g_.resize(3 * FloatingBase::Foot::foot_count * planning_horizon_, 1);
  C_.resize(num_constraints_ * FloatingBase::Foot::foot_count * planning_horizon_, 3 * FloatingBase::Foot::foot_count * planning_horizon_);
  constraint_lb_.resize(num_constraints_ * FloatingBase::Foot::foot_count * planning_horizon_, 1);
  constraint_ub_.resize(num_constraints_ * FloatingBase::Foot::foot_count * planning_horizon_, 1);

  H_.setZero();
  g_.setZero();
  C_.setZero();
  std::cout << "done initializing the controller" << std::endl;
}

void ConvexMpc::Compute() {
  // First we get the foot positions expressed in the world frame.
  // The foot position vectors are defined as the as the vector from the
  // body base frame origin B0 to the foot frame origin (e.g. FL0 for front left).

  // Wait for the robot pose to be updated.
  if (quadruped_.foot_poses().size() == 0) { 
    std::cout << "Pose not updated. Exiting control computation." << std::endl;
    return;
  } else {
    std::cout << "Pose set! Good to go." << std::endl;
  }
  quadruped_.UpdateDynamics();
  quadruped_.DiscretizeDynamics();
  ZeroTrajectory();
  std::cout << "---------- ref trajectory ----------" << std::endl;
  for (const auto& state : plan_trajectory_) {
    std::cout << "state" << std::endl;
    std::cout << "state x:" << state.x << std::endl;
    std::cout << "state y:" << state.y << std::endl;
    std::cout << "state z:" << state.z << std::endl;
    std::cout << "state x_dot:" << state.x_dot << std::endl;
    std::cout << "state y_dot:" << state.y_dot << std::endl;
    std::cout << "state z_dot:" << state.z_dot << std::endl;
    std::cout << "state roll:" << state.roll << std::endl;
    std::cout << "state pitch:" << state.pitch << std::endl;
    std::cout << "state yaw:" << state.yaw << std::endl;
    std::cout << "state roll_dot:" << state.roll_dot << std::endl;
    std::cout << "state pitch_dot:" << state.pitch_dot << std::endl;
    std::cout << "state yaw_dot:" << state.yaw_dot << std::endl;
    std::cout << "state g:" << state.g << std::endl;
  }
  CondensedFormulation();

  // convert to qpoases matricies
  qpOASES::real_t H_qpoases[H_.rows() * H_.cols()];
  qpOASES::real_t g_qpoases[g_.rows() * g_.cols()];
  qpOASES::real_t C_qpoases[C_.rows() * C_.cols()];
  qpOASES::real_t constraint_ub_qpoases[constraint_ub_.rows() * constraint_ub_.cols()];
  qpOASES::real_t constraint_lb_qpoases[constraint_lb_.rows() * constraint_lb_.cols()];
  qpOASES::real_t qp_solution[3 * FloatingBase::Foot::foot_count * planning_horizon_];
  
  MatrixToReal(H_qpoases, H_);
  MatrixToReal(g_qpoases, g_);
  MatrixToReal(C_qpoases, C_);
  MatrixToReal(constraint_ub_qpoases, constraint_ub_);
  MatrixToReal(constraint_lb_qpoases, constraint_lb_);

  std::cout << "------------ constraint_ub_qpoases -----------" << std::endl;
  for (size_t i = 0; i < constraint_ub_.rows() * constraint_ub_.cols(); ++i) {
    std::cout << constraint_ub_qpoases[i] << std::endl;
  }
  std::cout << "------------ constraint_lb_qpoases -----------" << std::endl;
  for (size_t i = 0; i < constraint_ub_.rows() * constraint_ub_.cols(); ++i) {
    std::cout << constraint_lb_qpoases[i] << std::endl;
  }

  const std::vector<int> foot_cs = ContactChecker(quadruped_.foot_poses());
  const int num_legs_in_contact = std::count(foot_cs.begin(), foot_cs.end(), 1);
  const qpOASES::int_t qp_dim = num_legs_in_contact * 3 * planning_horizon_;
  const qpOASES::int_t constraint_dim = num_legs_in_contact * num_constraints_ * planning_horizon_;
  qpOASES::QProblem qp_problem(qp_dim, constraint_dim, qpOASES::HST_UNKNOWN, qpOASES::BT_TRUE);
  
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_HIGH;
  qp_problem.setOptions(options);
  qpOASES::int_t nWSR = 100;
  qp_problem.init(H_qpoases, g_qpoases, C_qpoases, NULL, NULL, constraint_lb_qpoases, constraint_ub_qpoases, nWSR, NULL);
  int qp_status = qp_problem.getPrimalSolution(qp_solution);
  if (qp_status == qpOASES::SUCCESSFUL_RETURN) {
    std::cout << "succes!!" << std::endl;
    for (int i = 0; i < planning_horizon_; ++i) {
      std::cout << "step: " << i << std::endl;
      std::cout << "FL fx: " << qp_solution[i + 0] << std::endl;
      std::cout << "FL fy: " << qp_solution[i + 1] << std::endl;
      std::cout << "FL fz: " << qp_solution[i + 2] << std::endl;
      std::cout << "FR fx: " << qp_solution[i + 3] << std::endl;
      std::cout << "FR fy: " << qp_solution[i + 4] << std::endl;
      std::cout << "FR fz: " << qp_solution[i + 5] << std::endl;
      std::cout << "RL fx: " << qp_solution[i + 6] << std::endl;
      std::cout << "RL fy: " << qp_solution[i + 7] << std::endl;
      std::cout << "RL fz: " << qp_solution[i + 8] << std::endl;
      std::cout << "RR fx: " << qp_solution[i + 9] << std::endl;
      std::cout << "RR fy: " << qp_solution[i + 10] << std::endl;
      std::cout << "RR fz: " << qp_solution[i + 11] << std::endl;
    }
  } else {
    std::cout << "QP solve unsuccesful..." << std::endl;
  }
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
    // desired_state.z = timestep_ * (i + 1) * desired_velocity.z();
    desired_state.z = 0.333697;
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

  std::vector<Eigen::Matrix<double,13,13>> powerMats(planning_horizon_);
  powerMats[0].setIdentity();
  for(int i = 1; i < planning_horizon_; i++) {
    powerMats[i] = quadruped_.A_dt() * powerMats[i-1];
  }

  std::cout << "---------- A_dt ----------" << std::endl;
  std::cout << quadruped_.A_dt() << std::endl;
  for (int i = 0; i < planning_horizon_; ++i) {
    A_qp.block(i * fsidx::state_count, 0, fsidx::state_count, fsidx::state_count) = powerMats[i];
  }

  // Set B
  for (int i = 0; i < planning_horizon_; ++i) {
    // Go across the columns.
    Eigen::Matrix<double, 13, 13> A_power;
    A_power.setIdentity();
    for (int j = 0; j < planning_horizon_; ++j) {
      // Go across the rows.
      if (j >= i) {
        int a_num = j - i;
        B_qp.block(j * fsidx::state_count, i * fc::control_count, fsidx::state_count, fc::control_count) = powerMats[a_num] * quadruped_.B_dt();
      }
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
  constraints.resize(num_constraints_, 3);
  constraints << -1, 0, quadruped_.mu(),
                  1, 0, quadruped_.mu(),
                  0, -1, quadruped_.mu(),
                  0, 1, quadruped_.mu(),
                  0, 0, 1;

  // constraints << quadruped_.mu(), 0, 1,
  //                 -quadruped_.mu(), 0, 1,
  //                 0, quadruped_.mu(), 1,
  //                 0, -quadruped_.mu(), 1,
  //                 0, 0, 1;
  
  for (int i = 0; i < planning_horizon_ * FloatingBase::Foot::foot_count; ++i) {
    C_.block(i * num_constraints_, i * 3, num_constraints_, 3) = constraints;
  }

  std::cout << "---------- C ----------" << std::endl;
  std::cout << C_ << std::endl;

  // std::vector<int> foot_cs = ContactChecker(quadruped_.foot_poses());
  std::vector<int> foot_cs{1, 1, 1, 1};
  std::cout << "foot size: " << foot_cs.size() << std::endl;
  for (const auto& cs : foot_cs) {
    std::cout << "foot cs: " << cs << std::endl;
  }

  const double fz_max = quadruped_.mass() * 9.81 / 4;
  std::cout << "fz max: " << fz_max << std::endl;

  for (int i = 0; i < planning_horizon_; ++i) {
    for (int j = 0; j < FloatingBase::Foot::foot_count; ++j) {
      const int row = (i * FloatingBase::Foot::foot_count + j) * num_constraints_;
      constraint_lb_[row] = 0;
      constraint_lb_[row + 1] = 0;
      constraint_lb_[row + 2] = 0;
      constraint_lb_[row + 3] = 0;
      // constraint_lb_[row + 4] =  fz_max / 10 * foot_cs[j];
      constraint_lb_[row + 4] =  0;

      const double friction_ub = (quadruped_.mu() + 1) * fz_max * foot_cs[j];
      constraint_ub_[row] = friction_ub;
      constraint_ub_[row + 1] = friction_ub;
      constraint_ub_[row + 2] = friction_ub;
      constraint_ub_[row + 3] = friction_ub;
      constraint_ub_[row + 4] = fz_max * foot_cs[j];
    }
  }

  std::cout << "---------- C_ub ----------" << std::endl;
  std::cout << constraint_ub_ << std::endl;
  std::cout << "---------- C_lb ----------" << std::endl;
  std::cout << constraint_lb_ << std::endl;
}

std::vector<int> ConvexMpc::ContactChecker(const std::vector<Eigen::Isometry3d>& foot_poses) {
  std::vector<int> foot_cs;
  foot_cs.reserve(FloatingBase::Foot::foot_count);
  for (const auto& foot : foot_poses) {
    int cs = 0;
    if (foot.translation().z() < 0.25) {
      // we have contact.
      cs = 1;
    }
    foot_cs.push_back(cs);
  }

  return foot_cs;
}

void ConvexMpc::MatrixToReal(qpOASES::real_t* dst, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> src) {
  int a = 0;
  for(int r = 0; r < src.rows(); r++) {
    for(int c = 0; c < src.cols(); c++) {
      dst[a] = src(r,c);
      a++;
    }
  }
}

void ConvexMpc::ZeroTrajectory() {
  std::cout << "Zero Traj" << std::endl;
  FloatingBase::State desired_state;
  plan_trajectory_.clear();
  plan_trajectory_.reserve(planning_horizon_);
  for (int i = 0; i < planning_horizon_; ++i) {
    // Project the velocity to get desired position.
    // Fill up a state.
    desired_state.x = 0;
    desired_state.y = 0;
    desired_state.z = 0.433697;
    desired_state.x_dot = 0;
    desired_state.y_dot = 0;
    desired_state.z_dot = 0; // Should be zero to stabilize the height.

    // Desired roll pitch yaw should be 0 for stabilization.
    desired_state.roll = 0.0;
    desired_state.pitch = 0.0;
    desired_state.yaw = 0.0;
    // Desired roll and pitch rate should be zero for stabilization.
    desired_state.roll_dot = 0;
    desired_state.pitch_dot = 0;
    desired_state.yaw_dot = 0;

    desired_state.g = -9.81; // m/s^2

    plan_trajectory_.push_back(desired_state);
  }
}