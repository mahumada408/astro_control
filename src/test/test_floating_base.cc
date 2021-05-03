#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include "astro_control/floating_base/floating_base.h"

// Declare a test
TEST(TestSuite, testCase1)
{
    // Test base state (position and velocity).
    std::cout << "Testing!" << std::endl;
    FloatingBase dut;

    Eigen::Isometry3d robo_pose;
    Eigen::Vector3d base_position;
    base_position << 1, 2, 3;

    Eigen::Vector3d lin_velocity;
    Eigen::Vector3d ang_velocity;
    lin_velocity << 1, 0, 0;
    ang_velocity << 0, 0, 0.1;

    Eigen::Matrix3d rot;
    rot << 0.7071068, -0.7071068,  0.0000000,
           0.7071068,  0.7071068,  0.0000000,
           0.0000000,  0.0000000,  1.0000000;
    Eigen::Vector3d euler_angles = rot.eulerAngles(0, 1, 2);

    robo_pose = Eigen::Translation3d(base_position.x(), base_position.y(), base_position.z()) * Eigen::Quaterniond(rot);

    dut.SetRobotPose(robo_pose, lin_velocity, ang_velocity);

    Eigen::Matrix<double, FloatingBase::State_idx::state_count, 1> test_state;

    test_state[FloatingBase::State_idx::x] = base_position.x();
    test_state[FloatingBase::State_idx::y] = base_position.y();
    test_state[FloatingBase::State_idx::z] = base_position.z();
    test_state[FloatingBase::State_idx::x_dot] = lin_velocity.x();
    test_state[FloatingBase::State_idx::y_dot] = lin_velocity.y();
    test_state[FloatingBase::State_idx::z_dot] = lin_velocity.z();
    test_state[FloatingBase::State_idx::roll] = euler_angles.x();
    test_state[FloatingBase::State_idx::pitch] = euler_angles.y();
    test_state[FloatingBase::State_idx::yaw] = euler_angles.z();
    test_state[FloatingBase::State_idx::roll_dot] = ang_velocity.x();
    test_state[FloatingBase::State_idx::pitch_dot] = ang_velocity.y();
    test_state[FloatingBase::State_idx::yaw_dot] = ang_velocity.z();
    test_state[FloatingBase::State_idx::g] = -9.81;

    std::cout << "test state" << std::endl;
    for (int i = 0; i < FloatingBase::State_idx::state_count; ++i) {
      std::cout << test_state[i] << std::endl;
    }

    std::cout << "robot state" << std::endl;
    auto robo_state = dut.robot_state();
    for (int i = 0; i < FloatingBase::State_idx::state_count; ++i) {
      std::cout << robo_state[i] << std::endl;
    }

    for (int i = 0; i < FloatingBase::State_idx::state_count; ++i) {
      ASSERT_NEAR(test_state[i], robo_state[i], 0.0001);
    }

    // Test foot position.
    Eigen::Isometry3d nominal_position;
    std::vector<Eigen::Isometry3d> foot_poses;
    foot_poses.reserve(FloatingBase::Foot::foot_count);
    Eigen::Matrix3d foot_rot;
    foot_rot << 1, 0,  0,
                0,  1,  0,
                0,  0,  1;

    // FL
    nominal_position = Eigen::Translation3d(0.174542, 0.127191, 0.0306211) * Eigen::Quaterniond(foot_rot);
    foot_poses.push_back(nominal_position);
    // FR
    nominal_position = Eigen::Translation3d(0.174542, -0.127191, 0.0306211) * Eigen::Quaterniond(foot_rot);
    foot_poses.push_back(nominal_position);
    // RL
    nominal_position = Eigen::Translation3d(-0.186458, 0.127191, 0.0306211) * Eigen::Quaterniond(foot_rot);
    foot_poses.push_back(nominal_position);
    // FL
    nominal_position = Eigen::Translation3d(-0.186458, -0.127191, 0.0306211) * Eigen::Quaterniond(foot_rot);
    foot_poses.push_back(nominal_position);

    dut.SetFootPositions(foot_poses);
    std::vector<Eigen::Isometry3d> robo_foot_poses = dut.foot_poses();

    for (int i = 0; i < FloatingBase::Foot::foot_count; ++i) {
      ASSERT_EQ(foot_poses[i].translation(), robo_foot_poses[i].translation());
    }

    

    std::cout << "All done" << std::endl;
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}