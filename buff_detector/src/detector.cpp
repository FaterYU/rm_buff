// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_detector/detector.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rm_buff
{
Detector::Detector(const std::string model_path) : model_path_(model_path)
{
  core_ = ov::Core();
  model_ = core_.read_model(model_path_);

  ov::preprocess::PrePostProcessor ppp(model_);
  ppp.input().preprocess().convert_layout({0, 3, 1, 2});
  // convert layout from [1, 18, 8400] to [1, 8400, 18]
  ppp.output().postprocess().convert_layout({0, 2, 1});
  model_ = ppp.build();

  compiled_model_ = core_.compile_model(model_, "CPU");
  infer_request_ = compiled_model_.create_infer_request();
  input_tensor_ = infer_request_.get_input_tensor(0);

  blade_template_ = {
    cv::Point2f(138.0, 0.0),   cv::Point2f(25.0, 28.0),   cv::Point2f(12.0, 125.0),
    cv::Point2f(5.0, 235.0),   cv::Point2f(0.0, 345.0),   cv::Point2f(155.0, 381.0),
    cv::Point2f(216.0, 381.0), cv::Point2f(371.0, 345.0), cv::Point2f(366.0, 235.0),
    cv::Point2f(359.0, 125.0), cv::Point2f(346.0, 28.0),  cv::Point2f(233.0, 0.0)};

  corner_template_ = {
    cv::Point2f(25.0, 28.0), cv::Point2f(0.0, 345.0), cv::Point2f(371.0, 345.0),
    cv::Point2f(346.0, 28.0)};

  cv::Point2f center(185.0, 186.0);

  // [-160.0, -225.0],[-166.0, -122.0],[165.0, -122.0],[159.0, -225.0]
  kpt_template_ = {
    cv::Point2f(26.0, 131.0), cv::Point2f(20.0, 234.0), cv::Point2f(351.0, 234.0),
    cv::Point2f(345.0, 131.0)};

  double ratio_x = 1.25;
  double ratio_y = 1.1;
  double move_x = 0.0;
  double move_y = 0.0;

  for (size_t i = 0; i < blade_template_.size(); ++i) {
    blade_template_[i].x = (blade_template_[i].x - center.x + move_x) * ratio_x + center.x;
    blade_template_[i].y = (blade_template_[i].y - center.y + move_y) * ratio_y + center.y;
  }
}

// Detector::Detector() {}

// Detector::~Detector() {}

std::vector<Blade> Detector::Detect(cv::Mat & src_img)
{
  cv::Mat img;

  // cv::resize(src_img, img, cv::Size(image_size, image_size));
  img = letterbox(src_img, image_size, image_size);

  img.convertTo(img, CV_32FC3, 1.0 / 255.0);

  if (img.isContinuous()) {
    img = img.reshape(1, 1);
  } else {
    img = img.clone().reshape(1, 1);
  }
  input_tensor_ =
    ov::Tensor(input_tensor_.get_element_type(), input_tensor_.get_shape(), img.ptr<float>());

  infer_request_.set_input_tensor(0, input_tensor_);

  // the following method need 10x+ time than above 20240120
  // auto data = input_tensor_.data<float>();
  // for (int h = 0; h < image_size; h++) {
  //   for (int w = 0; w < image_size; w++) {
  //     for (int c = 0; c < 3; c++) {
  //       int out_index = c * image_size * image_size + h * image_size + w;
  //       data[out_index] = float(img.at<cv::Vec3f>(h, w)[c]);
  //     }
  //   }
  // }

  infer_request_.infer();
  // infer_request_.start_async();
  // infer_request_.wait();
  auto output = infer_request_.get_output_tensor(0);

  non_max_suppression(output, conf_threshold, nms_threshold, CLS_NUM, src_img.size());

  for (size_t i = 0; i < blade_array_.size(); i++) {
    if (!calibrate_kpts(blade_array_[i], src_img)) {
      blade_array_.erase(blade_array_.begin() + i);
      i--;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid keypoints");
    }
  }

  return blade_array_;
}

void Detector::non_max_suppression(
  ov::Tensor & output, float conf_thres, float iou_thres, int nc, cv::Size img_size)
{
  auto data = output.data<float>();

  int bs = output.get_shape()[0];  // batch size
  int nm = output.get_shape()[1];  // number of masks
  int ml = output.get_shape()[2];  // mask length

  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> kpts_list;
  std::vector<int> picked;
  std::vector<float> picked_useless;  // SoftNMS

  for (int i = 0; i < bs; i++) {
    for (int j = 0; j < nm; j++) {
      int offset = i * nm * ml + j * ml;
      float conf = 0.0;
      int classId = 0;
      for (int k = 0; k < nc; k++) {
        if (data[offset + 4 + k] > conf) {
          conf = data[offset + 4 + k];
          classId = k;
        }
      }
      if (conf < conf_thres) {
        continue;
      }
      float x = data[offset + 0];
      float y = data[offset + 1];
      float w = data[offset + 2];
      float h = data[offset + 3];

      classIds.emplace_back(classId);
      confidences.emplace_back(conf);
      boxes.emplace_back(cv::Rect(x, y, w, h));
      std::vector<cv::Point2f> kpts;
      for (int k = 0; k < KPT_NUM; k++) {
        cv::Point2f kpt;
        kpt.x =
          (data[offset + 4 + nc + k * 2] - padd_w_) / (image_size - 2 * padd_w_) * img_size.width;
        kpt.y = (data[offset + 4 + nc + k * 2 + 1] - padd_h_) / (image_size - 2 * padd_h_) *
                img_size.height;
        kpts.emplace_back(kpt);
      }
      kpts_list.emplace_back(kpts);
    }
  }

  cv::dnn::softNMSBoxes(boxes, confidences, picked_useless, conf_thres, iou_thres, picked);

  blade_array_.clear();
  for (size_t i = 0; i < picked.size(); ++i) {
    Blade blade;
    int idx = picked[i];
    auto x = float(boxes[idx].x - padd_w_) / (image_size - 2 * padd_w_) * img_size.width;
    auto y = float(boxes[idx].y - padd_h_) / (image_size - 2 * padd_h_) * img_size.height;
    auto width = float(boxes[idx].width) / image_size * img_size.width;
    auto height = float(boxes[idx].height) / image_size * img_size.height;
    blade.rect = cv::Rect(x - (width / 2), y - (height / 2), width, height);
    blade.label = classIds[idx];
    blade.prob = confidences[idx];
    blade.kpt = kpts_list[idx];
    blade_array_.emplace_back(blade);
  }

  return;
}

bool Detector::calibrate_kpts(Blade & blade, cv::Mat & img)
{
  debug_img = img.clone();
  // cv::rectangle(debug_img, blade.rect, cv::Scalar(0, 255, 0), 4);

  cv::Point2f outP, inP, center;
  outP.x = (blade.kpt[0].x + blade.kpt[4].x) / 2;
  outP.y = (blade.kpt[0].y + blade.kpt[4].y) / 2;
  inP.x = (blade.kpt[1].x + blade.kpt[3].x) / 2;
  inP.y = (blade.kpt[1].y + blade.kpt[3].y) / 2;
  center.x = (outP.x + inP.x) / 2;
  center.y = (outP.y + inP.y) / 2;

  auto orient_angle = atan2(inP.y - outP.y, inP.x - outP.x) * 180 / M_PI + 45;
  cv::Mat rot_mat = cv::getRotationMatrix2D(center, orient_angle, 1);
  cv::Mat inv_rot_mat;
  cv::invertAffineTransform(rot_mat, inv_rot_mat);

  std::vector<cv::Point2f> dstPoints;
  dstPoints = {blade.kpt[0], blade.kpt[1], blade.kpt[3], blade.kpt[4]};
  auto M = cv::getPerspectiveTransform(kpt_template_, dstPoints);

  // blade_template_ in M
  std::vector<cv::Point2f> dstBlade;
  cv::perspectiveTransform(blade_template_, dstBlade, M);

  // corner_template_ in M
  std::vector<cv::Point2f> dstCorner;
  cv::perspectiveTransform(corner_template_, dstCorner, M);

  // 最小外接矩形
  cv::RotatedRect dstRect = cv::minAreaRect(dstBlade);

  std::vector<cv::Point2f> dstRectP;
  cv::Point2f vertices[4];
  dstRect.points(vertices);
  for (int i = 0; i < 4; i++) {
    dstRectP.emplace_back(vertices[i]);
  }

  for (size_t i = 0; i < dstRectP.size(); ++i) {
    cv::line(debug_img, dstRectP[i], dstRectP[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);
  std::vector<cv::Point> blade_contour(dstRectP.begin(), dstRectP.end());
  std::vector<std::vector<cv::Point>> roi_corners = {blade_contour};

  cv::fillPoly(mask, roi_corners, cv::Scalar(255, 255, 255));
  cv::Mat masked_img;
  cv::bitwise_and(img, mask, masked_img);

  cv::Mat gray_img;
  cv::cvtColor(masked_img, gray_img, cv::COLOR_BGR2GRAY);
  cv::threshold(gray_img, binary_img, bin_threshold, 255, cv::THRESH_BINARY);
  cv::Mat rotated_img;
  cv::warpAffine(masked_img, rotated_img, rot_mat, img.size());

  // process the keypoints
  cv::Mat gray_rotated_img;
  cv::cvtColor(rotated_img, gray_rotated_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_rotated_img;
  cv::threshold(gray_rotated_img, binary_rotated_img, bin_threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(
    binary_rotated_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> merged_contour;
  for (size_t i = 0; i < contours.size(); ++i) {
    for (size_t j = 0; j < contours[i].size(); ++j) {
      merged_contour.emplace_back(contours[i][j]);
    }
  }

  cv::Rect rect = cv::boundingRect(merged_contour);
  double u, d, l, r;
  u = rect.y;
  d = rect.y + rect.height;
  l = rect.x;
  r = rect.x + rect.width;

  std::vector<cv::Point2f> roi_points(4, cv::Point2f(0.0, 0.0));
  std::vector<double> roi_points_diff(4, 0.0);
  std::vector<int> coincidenceB(4, 0);

  for (auto & point : merged_contour) {
    int k = -1;
    if (d - 1 == point.y) {
      k = 0;
    } else if (r - 1 == point.x) {
      k = 1;
    } else if (u == point.y) {
      k = 2;
    } else if (l == point.x) {
      k = 3;
    }
    if (k != -1) {
      if (coincidenceB[k] == 0) {
        roi_points[k] = static_cast<cv::Point2f>(point);
        coincidenceB[k] = 1;
      } else {
        roi_points[k] = (roi_points[k] + static_cast<cv::Point2f>(point)) / 2;
      }
    }
  }

  for (size_t i = 0; i < roi_points.size(); ++i) {
    cv::Point2f p;
    p.x = roi_points[i].x;
    p.y = roi_points[i].y;
    cv::Mat point = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
    point = inv_rot_mat * point;
    cv::circle(
      debug_img, cv::Point(point.at<double>(0), point.at<double>(1)), 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(
      debug_img, cv::Point(dstCorner[i].x, dstCorner[i].y), 3, cv::Scalar(255, 255, 0), -1);
    roi_points[i].x = point.at<double>(0);
    roi_points[i].y = point.at<double>(1);
    roi_points_diff[i] = cv::norm(roi_points[i] - dstCorner[i]);
  }

  double kpt_width =
    (cv::norm(dstCorner[0] - dstCorner[3]) + cv::norm(dstCorner[1] - dstCorner[2])) / 2;
  double kpt_height =
    (cv::norm(dstCorner[0] - dstCorner[1]) + cv::norm(dstCorner[2] - dstCorner[3])) / 2;
  double fault_tolerant = pow(pow(kpt_width, 2) + pow(kpt_height, 2), 0.5) * fault_tolerance_ratio;

  blade.kpt[0] = roi_points[0];
  blade.kpt[1] = roi_points[1];
  blade.kpt[3] = roi_points[2];
  blade.kpt[4] = roi_points[3];

  for (auto & diff : roi_points_diff) {
    if (diff > fault_tolerant) {
      return false;
    }
  }

  return true;
}

cv::Mat Detector::letterbox(cv::Mat & src, int h, int w)
{
  int in_w = src.cols;  // width
  int in_h = src.rows;  // height
  int tar_w = w;
  int tar_h = h;
  float r = std::min(float(tar_h) / in_h, float(tar_w) / in_w);
  int inside_w = round(in_w * r);
  int inside_h = round(in_h * r);
  padd_w_ = tar_w - inside_w;
  padd_h_ = tar_h - inside_h;

  cv::Mat resize_img;

  cv::resize(src, resize_img, cv::Size(inside_w, inside_h));

  padd_w_ = padd_w_ / 2;
  padd_h_ = padd_h_ / 2;

  int top = int(round(padd_h_ - 0.1));
  int bottom = int(round(padd_h_ + 0.1));
  int left = int(round(padd_w_ - 0.1));
  int right = int(round(padd_w_ + 0.1));
  cv::copyMakeBorder(
    resize_img, resize_img, top, bottom, left, right, 0, cv::Scalar(114, 114, 114));

  return resize_img;
}

void Detector::draw_blade(cv::Mat & img)
{
  for (size_t i = 0; i < blade_array_.size(); ++i) {
    int kpt_idx[4] = {0, 1, 3, 4};
    for (int j = 0; j < 2; j++) {
      auto kpt_start =
        cv::Point(blade_array_[i].kpt[kpt_idx[j]].x, blade_array_[i].kpt[kpt_idx[j]].y);
      auto kpt_end =
        cv::Point(blade_array_[i].kpt[kpt_idx[j + 2]].x, blade_array_[i].kpt[kpt_idx[j + 2]].y);
      cv::line(img, kpt_start, kpt_end, cv::Scalar(0, 255, 0), 4);
    }
    cv::circle(
      img, cv::Point(blade_array_[i].kpt[2].x, blade_array_[i].kpt[2].y), 8, cv::Scalar(255, 0, 0),
      -1);
  }
}

}  // namespace rm_buff
