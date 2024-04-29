#pragma once

#include <string>
#include <vector>

namespace mumei {
namespace utils {

std::vector<std::string> split(const std::string& str,
                               const std::string& delimiter);
}  // namespace utils
}  // namespace mumei
