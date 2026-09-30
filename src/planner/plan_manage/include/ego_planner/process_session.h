#pragma once
#include <array>
#include <iomanip>
#include <random>
#include <sstream>

namespace ego_planner {
inline std::string newProcessSession() {
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) out << std::setw(8) << random();
  return out.str();
}
}
