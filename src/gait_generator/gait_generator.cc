#include <astro_control/gait_generator/gait_generator.h>
#include <astro_control/floating_base/floating_base.h>

#include <cmath>

namespace gait_generator {

std::vector<std::vector<double>> GenerateGait(int planning_horizon, std::vector<double> input_phase, double t) {
  double tp = 0.5;
  double switch_phase = 0.5;
  
  std::vector<std::vector<double>> all_phases;
  std::vector<std::vector<double>> all_gaits;
  all_phases.reserve(planning_horizon);
  all_gaits.reserve(planning_horizon);

  for (int i = 0; i < planning_horizon; ++i) {
    std::vector<double> current_phase;
    std::vector<double> current_gait;
    current_phase.reserve(FloatingBase::Foot::foot_count);
    current_gait.reserve(FloatingBase::Foot::foot_count + 1);
    double phase = std::fmod(t / tp, 1.0);
    for (int j = 0; j < FloatingBase::Foot::foot_count; ++j) {
      double foot_phase = phase + input_phase[j];
      double foot_wrapped = std::fmod(foot_phase, 1);
      double stance = foot_wrapped < switch_phase ? 0 : 1;
      current_phase.push_back(foot_wrapped);
      current_gait.push_back(stance);
    }
    current_gait.push_back(t);
    all_phases.push_back(current_phase);
    all_gaits.push_back(current_gait);
    t += 0.03;
  }

  return all_gaits;
}

} // namespace gait_generator