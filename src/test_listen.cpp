#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/LinkStates.h>

#include "astro_control/convex_mpc/convex_mpc_interface.h"

#include <cmath>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
 

  ConvexMpcInterface mpc_interface(10, 0.03);

  // Subscribe to gazebo poses.
  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, &ConvexMpcInterface::GazeboPoses, &mpc_interface);

  // Subscribe to keyboard inputs.
  ros::Subscriber sub2 = n.subscribe("astro_key/cmd_vel", 1000, &ConvexMpcInterface::InputToTrajectory, &mpc_interface);

  // Controller runs at 100 Hz.
  // ros::Timer timer = n.createTimer(ros::Duration(0.01), &ConvexMpcInterface::Compute, &mpc_interface);

  ros::spin();


  return 0;
}