#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include "astro_control/convex_mpc/convex_mpc.h"


// Declare a test
TEST(TestSuite, testCase1)
{
    std::cout << "Testing!" << std::endl;
    ConvexMpc controller(2, 0.03);

    
    // Update robot pose.
    std::vector<Eigen::Isometry3d> foot_poses;
    Eigen::Isometry3d foot;
    Eigen::Vector3d foot_position;
    Eigen::Matrix3d foot_rotation;

    foot_poses.resize(FloatingBase::Foot::foot_count);
    
    // FL
    foot_position << 0.1431336694, -0.3343344813, 0.01997319203;
    foot_rotation << 0.2945387836, -0.9395692033, -0.1745182431,
                     0.7373604909, 0.3396091879, -0.5839221745,
                     0.6079032911, 0.04330486959, 0.7928292861;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses[FloatingBase::Foot::fl] = foot;

    // FR
    foot_position << 0.4101086254, -0.4173683127, 0.01994888275;
    foot_rotation << 0.2561523748, -0.9397538778, -0.2263815585,
                     0.7910349235, 0.3383903407, -0.5096623658,
                     0.5555625173, -0.04852449351, 0.8300576262;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses[FloatingBase::Foot::fr] = foot;

    // RL
    foot_position << 0.03108941724, -0.6719744509, 0.01994952366;
    foot_rotation << 0.275175952, -0.9407147085, -0.1983280933,
                     0.7664811701, 0.3391932947, -0.5453939169,
                     0.580331639, -0.001935458721, 0.8143779484;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses[FloatingBase::Foot::rl] = foot;

    // RR
    foot_position << 0.2726257793, -0.7566161904, 0.01997893463;
    foot_rotation << 0.2709639413, -0.9406331747, -0.2044205792,
                     0.7459169735, 0.3394133047, -0.5730676026,
                     0.6084294627, 0.002799876504, 0.7936030176;
    foot.translation() = foot_position;
    foot.linear() = foot_rotation;
    foot_poses[FloatingBase::Foot::rr] = foot;

    // base
    Eigen::Isometry3d base;
    Eigen::Vector3d base_position;
    Eigen::Matrix3d base_rotation;
    base_position << 0.3745211273, -0.4267566197, 0.3348462036;
    base_rotation << 0.3393212571, -0.9406654808, 0.003088333849,
                     0.9405990548, 0.3393341159, 0.01121498606,
                    -0.01159752729, -0.0009005992683, 0.9999323409;
    base.translation() = base_position;
    base.linear() = base_rotation;

    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;

    linear_vel << -0.0001110495528, -0.0002753581191, -0.0004400001701;
    angular_vel << 0.0004411750019, -8.867135708e-05, -0.0005540135833;

    // linear_vel.setZero();
    // angular_vel.setZero();

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