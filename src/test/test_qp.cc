#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include "astro_control/convex_mpc/convex_mpc.h"


// Declare a test
TEST(TestSuite, testCase1)
{
    std::cout << "Testing!" << std::endl;
    ConvexMpc controller(10, 0.03);

    
    // Update robot pose.
    std::vector<Eigen::Isometry3d> foot_poses;
    Eigen::Isometry3d foot;
    Eigen::Vector3d foot_position;
    Eigen::Matrix3d foot_rotation;

    foot_poses.resize(FloatingBase::Foot::foot_count);
    
    // FL
    foot_position << 0.174542, 0.127191, 0.0306211;
    foot_rotation << 0.807443, 0, -0.589946,
                     0.00667612, 0.999936, 0.00913769,
                     0.589908, -0.0113167, 0.807391;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    // foot_poses[FloatingBase::Foot::fl] = foot;
    foot_poses[FloatingBase::Foot::fl] = Eigen::Translation3d(foot_position.x(), foot_position.y(), foot_position.z()) * Eigen::Quaterniond(foot_rotation);

    // FR
    foot_position << 0.174542, -0.127191, 0.0306211;
    foot_rotation << 0.807443, 0, -0.589946,
                    -0.00667691, 0.999936, -0.00913838,
                     0.589908, 0.0113177, 0.807391;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    // foot_poses[FloatingBase::Foot::fr] = foot;
    foot_poses[FloatingBase::Foot::fr] = Eigen::Translation3d(foot_position.x(), foot_position.y(), foot_position.z()) * Eigen::Quaterniond(foot_rotation);

    // RL
    foot_position << -0.186458, 0.127191, 0.0306211;
    foot_rotation << 0.807443, 0, -0.589946,
                     0.00667461, 0.999936, 0.00913642,
                     0.589908, -0.0113148, 0.807391;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    // foot_poses[FloatingBase::Foot::rl] = foot;
    foot_poses[FloatingBase::Foot::rl] = Eigen::Translation3d(foot_position.x(), foot_position.y(), foot_position.z()) * Eigen::Quaterniond(foot_rotation);

    // RR
    foot_position << -0.186458, -0.127191, 0.0306211;
    foot_rotation << 0.807443, 0, -0.589946,
                    -0.00667493, 0.999936, -0.00913664,
                     0.589908, 0.0113152, 0.807391;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    // foot_poses[FloatingBase::Foot::rr] = foot;
    foot_poses[FloatingBase::Foot::rr] = Eigen::Translation3d(foot_position.x(), foot_position.y(), foot_position.z()) * Eigen::Quaterniond(foot_rotation);

    // base
    Eigen::Isometry3d base;
    Eigen::Vector3d base_position;
    Eigen::Matrix3d base_rotation;
    base_position << 0, 0, 0.333697;
    base_rotation << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;
    // base.translation() = base_position;
    // base.linear() = base_rotation;
    base = Eigen::Translation3d(base_position.x(), base_position.y(), base_position.z()) * Eigen::Quaterniond(base_rotation);

    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;

    linear_vel << -0.0001110495528, -0.0002753581191, -0.0004400001701;
    angular_vel << 0.0004411750019, -8.867135708e-05, -0.0005540135833;

    // linear_vel.setZero();
    // angular_vel.setZero();

    controller.UpdateRobotPose(foot_poses, base, linear_vel, angular_vel);
    controller.Compute();


    ASSERT_TRUE(true) << "failed the test of champions";

    std::cout << "All done" << std::endl;
    
}

// Test the condensed matricies to make sure they are combined appropriately.
TEST(TestSuite, TestCondensedForm) {
    
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