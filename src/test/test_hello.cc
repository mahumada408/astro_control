#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include "astro_control/convex_mpc/convex_mpc.h"


// Declare a test
TEST(TestSuite, testCase1)
{
    std::cout << "Testing!" << std::endl;
    ConvexMpc controller(2, 0.25);

    
    // Update robot pose.
    std::vector<Eigen::Isometry3d> foot_poses;
    Eigen::Isometry3d foot;
    Eigen::Vector3d foot_position;
    Eigen::Matrix3d foot_rotation;

    foot_poses.reserve(FloatingBase::Foot::foot_count);

    // FL
    foot_position << -0.0615262, 0.1308, 0.28163;
    foot_rotation << 0.862401, 0.108549, -0.494451,
                    -0.0912023, 0.994073, 0.0591624,
                     0.497943, -0.00592663, 0.86719;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses.push_back(foot);

    // FR
    foot_position << -0.0615262, -0.1308, 0.28163;
    foot_rotation << 0.862401, 0.108549, -0.494451,
                    -0.0912023, 0.994073, 0.0591624,
                     0.497943, -0.00592663, 0.86719;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses.push_back(foot);

    // RL
    foot_position << -0.422526, 0.1308, 0.28163;
    foot_rotation << -0.207025, 0.108549, -0.972295,
                      0.028436, 0.994073, 0.104926,
                      0.977922, -0.00592661, -0.208885;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses.push_back(foot);

    // RR
    foot_position << -0.422526, -0.1308, 0.28163;
    foot_rotation << -0.740104, 0.108549, -0.663674,
                      0.084794, 0.994073, 0.0680295,
                      0.667125, -0.00592661, -0.744922;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses.push_back(foot);

    // base
    Eigen::Isometry3d base;
    Eigen::Vector3d base_position;
    Eigen::Matrix3d base_rotation;
    base_position << 0, 0, 0.6;
    base_rotation << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;

    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;

    linear_vel.setZero();
    angular_vel.setZero();

    controller.UpdateRobotPose(foot_poses, base, linear_vel, angular_vel);
    controller.Compute();


    ASSERT_TRUE(false) << "failed the test of champions";

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