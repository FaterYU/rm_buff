// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_DETECTOR__PNP_SOLVER_HPP_
#define BUFF_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "buff_detector/blade.hpp"

namespace rm_buff
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Blade & blade, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between blade center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Unit: mm
  // static constexpr float BLADE_TOP_WIDTH = 323.81;
  // static constexpr float BLADE_BOTTOM_WIDTH = 332.78;
  // static constexpr float BLADE_TOP_HEIGHT = 55.76;
  // static constexpr float BLADE_BOTTOM_HEIGHT = 47.91;
  static constexpr float BLADE_TOP_WIDTH = 319.0;
  static constexpr float BLADE_BOTTOM_WIDTH = 372.0;
  static constexpr float BLADE_TOP_HEIGHT = 157.0;
  static constexpr float BLADE_BOTTOM_HEIGHT = 159.0;
  static constexpr float BLADE_RADIUS = 700.70;

  // Four vertices of blade in 3d
  std::vector<cv::Point3f> blade_points_;
};

}  // namespace rm_buff

#endif  // BUFF_DETECTOR__PNP_SOLVER_HPP_