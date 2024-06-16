// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_buff
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double half_top_y = BLADE_TOP_WIDTH / 2 / 1000;
  constexpr double half_bottom_y = BLADE_BOTTOM_WIDTH / 2 / 1000;
  constexpr double half_top_z = BLADE_TOP_HEIGHT / 1000;
  constexpr double half_bottom_z = BLADE_BOTTOM_HEIGHT / 1000;
  // constexpr double radius = BLADE_RADIUS / 1000;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  blade_points_.emplace_back(cv::Point3f(0, half_bottom_y, -half_bottom_z));
  blade_points_.emplace_back(cv::Point3f(0, half_bottom_y, half_bottom_z));
  blade_points_.emplace_back(cv::Point3f(0, -half_top_y, half_top_z));
  blade_points_.emplace_back(cv::Point3f(0, -half_top_y, -half_top_z));
}

bool PnPSolver::solvePnP(const Blade & blade, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_blade_points;

  // Fill in image points
  image_blade_points.emplace_back(cv::Point2f(blade.kpt[1].x, blade.kpt[1].y));
  image_blade_points.emplace_back(cv::Point2f(blade.kpt[0].x, blade.kpt[0].y));
  image_blade_points.emplace_back(cv::Point2f(blade.kpt[4].x, blade.kpt[4].y));
  image_blade_points.emplace_back(cv::Point2f(blade.kpt[3].x, blade.kpt[3].y));

  // Solve pnp
  return cv::solvePnP(
    blade_points_, image_blade_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_buff