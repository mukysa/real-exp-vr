// Offline probe of the existing GELLO reference generator. No ROS or device IO.
#include "franka_fr3_arm_controllers/joint_reference_generator.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using Generator = franka_fr3_arm_controllers::JointReferenceGenerator;

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::ofstream trace(argv[1]);
  if (!trace) return 3;
  trace << "time_s,following";
  for (int j=1; j<=7; ++j) trace << ",controller_reference_j" << j;
  trace << '\n' << std::setprecision(17);
  Generator generator;
  Generator::Vector target{};
  double stamp, previous=0, clock=0, vmax=0, amax=0, lag=0;
  bool following, was_following=false, initialized=false;
  std::size_t ticks=0, violations=0, packets=0;
  auto advance = [&](double duration) {
    while (duration > 1e-10) {
      const double dt=std::min(.001, duration);
      generator.advance(dt);
      clock += dt;
      const auto& s=generator.state();
      trace << clock << ',' << was_following;
      for (int j=0; j<7; ++j) {
        trace << ',' << s.position[j];
        vmax=std::max(vmax, std::abs(s.velocity[j]));
        amax=std::max(amax, std::abs(s.acceleration[j]));
        if (!std::isfinite(s.position[j]) || !std::isfinite(s.velocity[j]) || !std::isfinite(s.acceleration[j]) ||
            s.position[j] < Generator::kPositionLower[j]-1e-9 || s.position[j] > Generator::kPositionUpper[j]+1e-9 ||
            std::abs(s.velocity[j]) > Generator::kMaxVelocity[j]+1e-9 ||
            std::abs(s.acceleration[j]) > Generator::kMaxAcceleration[j]+1e-9) ++violations;
      }
      trace << '\n';
      duration -= dt;
      ++ticks;
    }
  };
  while (std::cin >> stamp >> following) {
    for (double& q : target) if (!(std::cin >> q)) return 4;
    if (!initialized) {
      generator.reset(target);
      previous=stamp;
      initialized=true;
    }
    if (stamp < previous) return 5;
    advance(stamp-previous);
    if (following) {
      if (!generator.setTarget(target)) return 6;
      for (int j=0; j<7; ++j)
        lag=std::max(lag,std::abs(target[j]-generator.state().position[j]));
    } else if (was_following) generator.requestStop();
    was_following=following;
    previous=stamp;
    ++packets;
  }
  generator.requestStop();
  was_following=false;
  advance(2.0);
  std::cout << std::setprecision(17)
            << "{\"packets\":" << packets << ",\"ticks\":" << ticks
            << ",\"constraint_violations\":" << violations
            << ",\"max_speed_rad_s\":" << vmax
            << ",\"max_acceleration_rad_s2\":" << amax
            << ",\"max_waypoint_to_controller_reference_lag_rad\":" << lag << "}\n";
  return violations ? 1 : 0;
}
