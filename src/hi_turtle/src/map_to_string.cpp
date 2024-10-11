#include "hi_turtle/map_to_string.hpp"
#include <sstream>

std::string mapToString(const std::unordered_map<std::string, std::string>& map) {
    std::ostringstream oss;
    for (const auto& pair : map) {
        oss << "\n(" << pair.first << ": " << pair.second << ")";
    }
    return oss.str(); // 返回格式化后的字符串
}