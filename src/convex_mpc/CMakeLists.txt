cmake_minimum_required(VERSION 3.0.2)

##############
## Building ##
##############

add_library(convex_mpc
            convex_mpc.cc
            convex_mpc_interface.cc)
target_link_libraries(convex_mpc
                      ${catkin_LIBRARIES}
                      floating_base
                      qpOASES)