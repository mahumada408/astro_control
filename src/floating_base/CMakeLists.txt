cmake_minimum_required(VERSION 3.0.2)

##############
## Building ##
##############

add_library(floating_base floating_base.cc)
target_link_libraries(floating_base ${catkin_LIBRARIES})