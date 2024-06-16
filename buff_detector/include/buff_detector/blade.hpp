// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_DETECTOR__BLADE_HPP_
#define BUFF_DETECTOR__BLADE_HPP_

#include <opencv2/core.hpp>

namespace rm_buff
{
struct Blade
{
  Blade() = default;
  cv::Rect rect;
  int label;
  float prob;
  std::vector<cv::Point2f> kpt;
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__BLADE_HPP_