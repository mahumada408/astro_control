#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Twist.h>

#include "astro_control/convex_mpc/convex_mpc.h"


class ConvexMpcInterface {
  public:
    ConvexMpcInterface(int planning_horizon, double timestep);

    // Gets the gazebo ground truth poses for the frames of interest.
    void GazeboPoses(const gazebo_msgs::LinkStates& msg);

    // Input to trajectory.
    // Gets the input from the keyboard and generates a trajectory.
    void InputToTrajectory(const geometry_msgs::Twist& msg);

    // The main compute call for the MPC.
    void Compute(const ros::TimerEvent& event);
  private:
    ConvexMpc controller_;

    Eigen::Isometry3d base_frame_;
    Eigen::Vector3d v_WBo_; // Bo's velocity in world frame.
    Eigen::Vector3d w_WB_; // B frame's angular velocity in W.
    std::vector<Eigen::Isometry3d> foot_poses_;
};