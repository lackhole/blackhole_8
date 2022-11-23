//
// Created by YongGyu Lee on 2022/11/24.
//

#ifndef BLACKHOLE_CONFIG_H_
#define BLACKHOLE_CONFIG_H_

#include <chrono>
#include <filesystem>
#include <string>

#include "opencv2/opencv.hpp"

#define BH_STRINGIFY_IMPL(x) #x
#define BH_STRINGIFY(x) BH_STRINGIFY_IMPL(x)

#ifdef BH_RESOURCE_DIR_INPUT
#define BH_RESOURCE_DIR BH_STRINGIFY(BH_RESOURCE_DIR_INPUT)
#else
#define BH_RESOURCE_DIR
#endif

#ifdef BH_OUTPUT_DIR_INPUT
#define BH_OUTPUT_DIR BH_STRINGIFY(BH_OUTPUT_DIR_INPUT)
#else
#define BH_OUTPUT_DIR
#endif

namespace blackhole {

inline std::filesystem::path resource_dir() {
  return BH_RESOURCE_DIR;
}

cv::Mat resource_image(const std::string& subpath, int flag = cv::IMREAD_COLOR) {
  return cv::imread(resource_dir()/subpath, flag);
}

inline std::filesystem::path output_dir() {
  return BH_OUTPUT_DIR;
}

inline std::filesystem::path timed_output_dir() {
  namespace fs = std::filesystem;
  using system_clock = std::chrono::system_clock;
  const auto cur_time = system_clock::to_time_t(system_clock::now());
  return output_dir()/std::to_string(cur_time);
}

} // namespace blackhole

#endif // BLACKHOLE_CONFIG_H_
