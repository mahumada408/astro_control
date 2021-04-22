#pragma once

#include <ros/ros.h>

#include <vector>

namespace gait_generator {

std::vector<std::vector<double>> GenerateGait(int planning_horizon, std::vector<double> input_phase, double t);

} // namespace gait_generator

