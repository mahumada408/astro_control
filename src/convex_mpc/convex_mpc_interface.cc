#include <chrono>
#include "astro_control/convex_mpc/convex_mpc_interface.h"
#include "astro_control/floating_base/floating_base.h"
#include <eigen_conversions/eigen_msg.h>

ConvexMpcInterface::ConvexMpcInterface(const int planning_horizon, const double timestep) : controller_(planning_horizon, timestep) {
  std::cout << "done initializing the interface" << std::endl;
}

void ConvexMpcInterface::GazeboPoses(const gazebo_msgs::LinkStates& msg) {
  foot_poses_.clear();
  foot_poses_.resize(FloatingBase::Foot::foot_count);
  
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
      tf::vectorMsgToEigen(msg.twist[i].linear, v_WBo_);
      tf::vectorMsgToEigen(msg.twist[i].angular, w_WB_);
      std::cout << "base" << std::endl;
      std::cout << base_frame_.translation() << std::endl;
      std::cout << base_frame_.rotation() << std::endl;
    }
    else if (msg.name[i] == "a1_gazebo::FL_calf") {
      // FL
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation() * foot;
      foot_eig.linear() = calf.rotation();
      foot_poses_[FloatingBase::Foot::fl] = foot_eig;
      std::cout << "calf fl" << std::endl;
      std::cout << calf.translation() << std::endl;
      std::cout << calf.rotation() << std::endl;
      std::cout << "fl" << std::endl;
      std::cout << foot_eig.translation() << std::endl;
      std::cout << foot_eig.rotation() << std::endl;
    }
    else if (msg.name[i] == "a1_gazebo::FR_calf") {
      // FR
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation() * foot;
      foot_eig.linear() = calf.rotation();
      foot_poses_[FloatingBase::Foot::fr] = foot_eig;
      std::cout << "calf fr" << std::endl;
      std::cout << calf.translation() << std::endl;
      std::cout << calf.rotation() << std::endl;
      std::cout << "fr" << std::endl;
      std::cout << foot_eig.translation() << std::endl;
      std::cout << foot_eig.rotation() << std::endl;
    }
    else if (msg.name[i] == "a1_gazebo::RL_calf") {
      // RL
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation() * foot;
      foot_eig.linear() = calf.rotation();
      foot_poses_[FloatingBase::Foot::rl] = foot_eig;
      std::cout << "calf rl" << std::endl;
      std::cout << calf.translation() << std::endl;
      std::cout << calf.rotation() << std::endl;
      std::cout << "rl" << std::endl;
      std::cout << foot_eig.translation() << std::endl;
      std::cout << foot_eig.rotation() << std::endl;
    }
    else if (msg.name[i] == "a1_gazebo::RR_calf") {
      // RR
      foot_eig.translation().setZero();
      tf::poseMsgToEigen(msg.pose[i], calf);
      foot_eig.translation() = calf.translation() + calf.rotation() * foot;
      foot_eig.linear() = calf.rotation();
      foot_poses_[FloatingBase::Foot::rr] = foot_eig;
      std::cout << "calf rr" << std::endl;
      std::cout << calf.translation() << std::endl;
      std::cout << calf.rotation() << std::endl;
      std::cout << "rr" << std::endl;
      std::cout << foot_eig.translation() << std::endl;
      std::cout << foot_eig.rotation() << std::endl;
    }
  }

  std::cout << "lin vel: " << v_WBo_ << std::endl;
  std::cout << "ang vel: " << w_WB_ << std::endl;

  controller_.UpdateRobotPose(foot_poses_, base_frame_, v_WBo_, w_WB_);
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
