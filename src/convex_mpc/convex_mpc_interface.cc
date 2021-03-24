#include <chrono>
#include "astro_control/convex_mpc/convex_mpc_interface.h"
#include "astro_control/floating_base/floating_base.h"
#include <eigen_conversions/eigen_msg.h>

ConvexMpcInterface::ConvexMpcInterface(const int planning_horizon, const double timestep) : controller_(planning_horizon, timestep) {
  foot_poses_.clear();
  foot_poses_.reserve(FloatingBase::Foot::foot_count);
}

void ConvexMpcInterface::GazeboPoses(const gazebo_msgs::LinkStates& msg) {
  // 0 | ground_plane::link
  // 1 | static_environment::floor
  // 2 | static_environment::floor2
  // 3 | static_environment::floor3
  // 4 | stairs::Stairs_1
  // 5 | a1_gazebo::base
  // 6 | a1_gazebo::FL_hip
  // 7 | a1_gazebo::FL_thigh
  // 8 | a1_gazebo::FL_calf
  // 9 | a1_gazebo::FR_hip
  // 10 | a1_gazebo::FR_thigh
  // 11 | a1_gazebo::FR_calf
  // 12 | a1_gazebo::RL_hip
  // 13 | a1_gazebo::RL_thigh
  // 14 | a1_gazebo::RL_calf
  // 15 | a1_gazebo::RR_hip
  // 16 | a1_gazebo::RR_thigh
  // 17 | a1_gazebo::RR_calf
  foot_poses_.clear();
  foot_poses_.reserve(FloatingBase::Foot::foot_count);
  
  // Couldnt find the position of the foot frames, so we're going to have to get the calf frame, then project the
  // foot frame from there. C is the calf frame, F is the foot frame.
  // p_NoFo_N = R_NC * p_NoFo_C
  Eigen::Isometry3d calf;
  Eigen::Vector3d foot;
  foot << 0, 0, -0.2;
  Eigen::Isometry3d foot_eig;

  // base frame
  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] == "a1_gazebo::base") {
      tf::poseMsgToEigen(msg.pose[i], base_frame_);
    }
    else if (msg.name[i] == "a1_gazebo::FL_calf") {
      // FL
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
      foot_eig.rotate(calf.rotation());
      foot_poses_[FloatingBase::Foot::fl] = foot_eig;
    }
    else if (msg.name[i] == "a1_gazebo::FR_calf") {
      // FR
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
      foot_eig.rotate(calf.rotation());
      foot_poses_[FloatingBase::Foot::fr] = foot_eig;
    }
    else if (msg.name[i] == "a1_gazebo::RL_calf") {
      // RL
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
      foot_eig.rotate(calf.rotation());
      foot_poses_[FloatingBase::Foot::rl] = foot_eig;
    }
    else if (msg.name[i] == "a1_gazebo::RR_calf") {
      // RR
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
      foot_eig.rotate(calf.rotation());
      foot_poses_[FloatingBase::Foot::rr] = foot_eig;
    }
  }

  // // FL
  // foot_eig.translation().setZero();
  // tf::poseMsgToEigen(msg.pose[8], calf);
  // foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
  // foot_eig.rotate(calf.rotation());
  // foot_poses_[FloatingBase::Foot::fl] = foot_eig;

  // // FR
  // foot_eig.translation().setZero();
  // tf::poseMsgToEigen(msg.pose[11], calf);
  // foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
  // foot_eig.rotate(calf.rotation());
  // foot_poses_[FloatingBase::Foot::fr] = foot_eig;

  // // RL
  // foot_eig.translation().setZero();
  // tf::poseMsgToEigen(msg.pose[14], calf);
  // foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
  // foot_eig.rotate(calf.rotation());
  // foot_poses_[FloatingBase::Foot::rl] = foot_eig;

  // // RR
  // foot_eig.translation().setZero();
  // tf::poseMsgToEigen(msg.pose[17], calf);
  // foot_eig.translation() = calf.translation() + calf.rotation().inverse() * foot;
  // foot_eig.rotate(calf.rotation());
  // foot_poses_[FloatingBase::Foot::rr] = foot_eig;

  controller_.UpdateRobotPose(foot_poses_, base_frame_);
}

void ConvexMpcInterface::InputToTrajectory(const geometry_msgs::Twist& msg) {
    // Convert geometry message to eigen vectors.
    Eigen::Vector3d desired_velocity;
    desired_velocity << msg.linear.x, msg.linear.y, msg.linear.z;

    Eigen::Vector3d desired_angular_velocity;
    desired_angular_velocity << msg.angular.x, msg.angular.y, msg.angular.z;

    controller_.GenerateTrajectory(desired_velocity, desired_angular_velocity);    
}

void ConvexMpcInterface::Compute(const ros::TimerEvent& event) {
  // Run the controller!
  controller_.Compute();
}
