cmake_minimum_required(VERSION 3.0.2)
project(astro_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf tf_conversions
  eigen_conversions
  sophus
)
# find_package(sophus REQUIRED)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

# Adds the individual libraries.
add_subdirectory(src/convex_mpc)
add_subdirectory(src/floating_base)
add_subdirectory(src/gait_generator)

add_executable(test_listen src/test_listen.cpp)
target_link_libraries(test_listen 
                      ${catkin_LIBRARIES}
                      floating_base
                      convex_mpc)

# Keyboard teleop
add_executable(astro_teleop_key src/astro_teleop.cc)
target_link_libraries(astro_teleop_key ${catkin_LIBRARIES})

# Unit tests
find_package(rostest REQUIRED)

add_rostest_gtest(astro_tests launch/gtest.test src/test/test_qp.cc)
target_link_libraries(astro_tests
                      ${catkin_LIBRARIES}
                      convex_mpc)

add_rostest_gtest(astro_tests launch/gtest.test src/test/test_qp.cc)
target_link_libraries(astro_tests
                      ${catkin_LIBRARIES}
                      convex_mpc)

add_rostest_gtest(gait_test launch/gtest_gait.test src/test/test_gait.cc)
target_link_libraries(gait_test
                      ${catkin_LIBRARIES}
                      gait_generator)
