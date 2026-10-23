// Deliberately read-only: no control, recovery, parameter or gripper calls.
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <chrono>
#include <iomanip>
#include <iostream>
int main(int argc, char** argv) {
  if(argc != 2) { std::cerr << "Usage: franka_readonly IP\n"; return 2; }
  try {
    franka::Robot robot(argv[1]);
    auto last=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    std::cout << std::setprecision(17);
    robot.read([&](const franka::RobotState& s) {
      auto now=std::chrono::steady_clock::now();
      if(now-last<std::chrono::milliseconds(20)) return true;
      last=now;
      const double host_monotonic_s =
          std::chrono::duration<double>(now.time_since_epoch()).count();
      std::cout << "{\"host_monotonic_s\":" << host_monotonic_s
                << ",\"robot_time_ms\":" << s.time.toMSec()
                << ",\"q\":[";
      for(int i=0;i<7;i++) std::cout << (i?",":"") << s.q[i];
      std::cout << "],\"dq\":[";
      for(int i=0;i<7;i++) std::cout << (i?",":"") << s.dq[i];
      std::cout << "],\"O_T_EE\":[";
      for(int i=0;i<16;i++) std::cout << (i?",":"") << s.O_T_EE[i];
      std::cout << "],\"F_T_EE\":[";
      for(int i=0;i<16;i++) std::cout << (i?",":"") << s.F_T_EE[i];
      std::cout << "]}" << std::endl;
      return true;
    });
  } catch(const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
}
