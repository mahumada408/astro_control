#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include <vector>

#include "astro_control/gait_generator/gait_generator.h"


// Declare a test
TEST(TestSuite, testCase1)
{
    std::cout << "Testing!" << std::endl;

    std::vector<double> current_phase{0, 0.5, 0.5, 0};
    std::vector<std::vector<double>> all_gaits = gait_generator::GenerateGait(10, current_phase, 0);

    for (const auto& gait : all_gaits) {
      for (const auto& elem : gait) {
        std::cout << elem << ", ";
      }
      std::cout << std::endl;
    }


    ASSERT_TRUE(true) << "failed the test of champions";

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