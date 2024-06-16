// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_DETECTOR__DETECTOR_HPP_
#define BUFF_DETECTOR__DETECTOR_HPP_

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "buff_detector/blade.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "openvino/openvino.hpp"

#define KPT_NUM 5
#define CLS_NUM 4

namespace rm_buff
{
class Detector
{
public:
  // Detector();
  Detector(const std::string model_path);
  // ~Detector();

  std::vector<Blade> Detect(cv::Mat & image);

  void draw_blade(cv::Mat & img);

  // param
  double nms_threshold;
  double conf_threshold;
  double image_size;
  double bin_threshold;
  double fault_tolerance_ratio;

  // visual
  cv::Mat binary_img;
  cv::Mat debug_img;

private:
  std::string model_path_;
  std::string bin_path_;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Tensor input_tensor_;

  int padd_w_ = 0;
  int padd_h_ = 0;

  std::vector<Blade> blade_array_;

  std::vector<cv::Point2f> blade_template_;
  std::vector<cv::Point2f> corner_template_;
  std::vector<cv::Point2f> kpt_template_;

  bool calibrate_kpts(Blade & blade, cv::Mat & img);

  cv::Mat letterbox(cv::Mat & src, int h, int w);

  void non_max_suppression(
    ov::Tensor & output, float conf_thres, float iou_thres, int nc, cv::Size img_size);

  const std::vector<std::string> class_names = {"RR", "RW", "BR", "BW"};
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__DETECTOR_HPP_
