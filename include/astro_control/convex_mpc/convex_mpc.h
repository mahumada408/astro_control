#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>

#include <eigen3/Eigen/Dense>
#include "astro_control/floating_base/floating_base.h"
#include "astro_control/qpOASES/include/qpOASES.hpp"


class ConvexMpc {
  public:
    ConvexMpc(int planning_horizon, double timestep);

    // This is the main calling function for the controller.
    // Here, we compute the contact forces for the quadruped using a convex MPC
    // formulation found here:
    //
    void Compute();

    // Generates the desired trajectory for the MPC.
    void GenerateTrajectory(const Eigen::Vector3d desired_velocity, const Eigen::Vector3d desired_angular_velocity);

    // Update robot pose.
    void UpdateRobotPose(const std::vector<Eigen::Isometry3d>& foot_poses, const Eigen::Isometry3d& base_pose, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity);

  private:
    // Here, we convert the A and B matricies to the A_qp and B_qp matricies.
    // Formulation for condenced QP can be found here:
    // http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.385.6703&rep=rep1&type=pdf
    void CondensedFormulation();

    std::vector<int> ContactChecker(const std::vector<Eigen::Isometry3d>& foot_poses);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> StateRef();

    void MatrixToReal(qpOASES::real_t* dst, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> src);

    void ZeroTrajectory();

    // Planning horizon.
    int planning_horizon_ = 0;

    // Timestep of the planning ane MPC trajectories.
    double timestep_ = 0;

    // Number of constraints.
    int num_constraints_ = 5;

    // Condensed QP formulation.
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> g_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C_;
    Eigen::VectorXd constraint_ub_;
    Eigen::VectorXd constraint_lb_;

    std::vector<FloatingBase::State> plan_trajectory_;
    FloatingBase quadruped_;

};