#include "buff_detector/detector.hpp"

namespace rm_buff {
Detector::Detector(const std::string model_path) : model_path_(model_path) {
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

  // [[138, 0], [25, 28], [12, 125], [5, 235], [0, 345], [155, 381], [216, 381],
  // [371, 345], [366, 235], [359, 125], [346, 28], [233, 0]]
  blade_template_ = {
      cv::Point(138, 0),   cv::Point(25, 28),   cv::Point(12, 125),
      cv::Point(5, 235),   cv::Point(0, 345),   cv::Point(155, 381),
      cv::Point(216, 381), cv::Point(371, 345), cv::Point(366, 235),
      cv::Point(359, 125), cv::Point(346, 28),  cv::Point(233, 0)};

  kpt_template_ = {cv::Point(26, 131), cv::Point(20, 234), cv::Point(351, 234),
                   cv::Point(345, 131)};
}

// Detector::Detector() {}

// Detector::~Detector() {}

std::vector<Blade> Detector::Detect(cv::Mat& src_img) {
  cv::Mat img;

  // cv::resize(src_img, img, cv::Size(image_size, image_size));
  img = letterbox(src_img, image_size, image_size);

  img.convertTo(img, CV_32FC3, 1.0 / 255.0);

  if (img.isContinuous()) {
    img = img.reshape(1, 1);
  } else {
    img = img.clone().reshape(1, 1);
  }
  input_tensor_ = ov::Tensor(input_tensor_.get_element_type(),
                             input_tensor_.get_shape(), img.ptr<float>());

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

  non_max_suppression(output, conf_threshold, nms_threshold, CLS_NUM,
                      src_img.size());

  for (auto& blade : blade_array_) {
    calibrate_kpts(blade, src_img);
  }

  return blade_array_;
}

void Detector::non_max_suppression(ov::Tensor& output, float conf_thres,
                                   float iou_thres, int nc, cv::Size img_size) {
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
        kpt.x = (data[offset + 4 + nc + k * 2] - padd_w_) /
                    (image_size - 2 * padd_w_) * img_size.width -
                img_size.width / 2;
        kpt.y = (data[offset + 4 + nc + k * 2 + 1] - padd_h_) /
                    (image_size - 2 * padd_h_) * img_size.height -
                img_size.height / 2;
        kpts.emplace_back(kpt);
      }
      kpts_list.emplace_back(kpts);
    }
  }

  cv::dnn::softNMSBoxes(boxes, confidences, picked_useless, conf_thres,
                        iou_thres, picked);

  blade_array_.clear();
  for (size_t i = 0; i < picked.size(); ++i) {
    Blade blade;
    int idx = picked[i];
    auto x = float(boxes[idx].x - padd_w_) / (image_size - 2 * padd_w_);
    auto y = float(boxes[idx].y - padd_h_) / (image_size - 2 * padd_h_);
    auto width = float(boxes[idx].width) / image_size;
    auto height = float(boxes[idx].height) / image_size;
    blade.rect = cv::Rect(x, y, width, height);
    blade.label = classIds[idx];
    blade.prob = confidences[idx];
    blade.kpt = kpts_list[idx];
    blade_array_.emplace_back(blade);
  }

  return;
}

void Detector::calibrate_kpts(Blade& blade, cv::Mat& img) {
  cv::Point2f center;
  center.x = (blade.rect.x + blade.rect.width / 2) * 2 - 1;
  center.y = (blade.rect.y + blade.rect.height / 2) * 2 - 1;
  auto orient_angle =
      atan2(blade.kpt[2].y - center.y, blade.kpt[2].x - center.x);
  auto rot_mat = cv::getRotationMatrix2D(center, orient_angle * 180 / M_PI, 1);
  auto inv_rot_mat = cv::getRotationMatrix2D(center, -orient_angle * 180 / M_PI,
                                             1);  // inverse rotation matrix

  std::vector<cv::Point2f> dstPoints;
  dstPoints = {blade.kpt[0], blade.kpt[1], blade.kpt[3], blade.kpt[4]};
  auto M = cv::getPerspectiveTransform(kpt_template_, dstPoints);

  // blade_template_ in M
  std::vector<cv::Point2f> dstBlade;
  cv::perspectiveTransform(blade_template_, dstBlade, M);

  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
  std::vector<cv::Point> blade_contour(dstBlade.begin(), dstBlade.end());
  std::vector<std::vector<cv::Point>> blade_contours = {blade_contour};

  cv::fillPoly(mask, blade_contours, cv::Scalar(255));
  cv::Mat masked_img;
  cv::bitwise_and(img, img, masked_img, mask);

  cv::Mat rotated_img;
  cv::warpPerspective(masked_img, rotated_img, rot_mat, img.size());

  // process the keypoints
  cv::Mat gray_rotated_img;
  cv::cvtColor(rotated_img, gray_rotated_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_rotated_img;
  cv::threshold(gray_rotated_img, binary_rotated_img, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_rotated_img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> merged_contour;
  for (size_t i = 0; i < contours.size(); ++i) {
    merged_contour.insert(merged_contour.end(), contours[i].begin(),
                          contours[i].end());
  }

  cv::Rect rect = cv::boundingRect(merged_contour);
  double u, d, l, r;
  u = rect.y;
  d = rect.y + rect.height;
  l = rect.x;
  r = rect.x + rect.width;

  std::vector<cv::Point> roi_points(4, cv::Point(0, 0));
  std::vector<int> coincidenceB(4, 0);

  for (auto& point : merged_contour) {
    int k = -1;
    if (r - 1 == point.x) {
      k = 0;
    } else if (d - 1 == point.y) {
      k = 1;
    } else if (l == point.x) {
      k = 2;
    } else if (u == point.y) {
      k = 3;
    }
    if (k != -1) {
      if (coincidenceB[k] == 0) {
        roi_points[k] = point;
        coincidenceB[k] = 1;
      } else {
        roi_points[k] = (roi_points[k] + point) / 2;
      }
    }
  }

  cv::perspectiveTransform(roi_points, blade.kpt, inv_rot_mat);
}

cv::Mat Detector::letterbox(cv::Mat& src, int h, int w) {
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
  cv::copyMakeBorder(resize_img, resize_img, top, bottom, left, right, 0,
                     cv::Scalar(114, 114, 114));

  return resize_img;
}

void Detector::draw_blade(cv::Mat& img) {
  for (size_t i = 0; i < blade_array_.size(); ++i) {
    int kpt_idx[4] = {0, 1, 3, 4};
    for (int j = 0; j < 2; j++) {
      auto kpt_start =
          cv::Point(blade_array_[i].kpt[kpt_idx[j]].x + (0.5 * img.cols),
                    blade_array_[i].kpt[kpt_idx[j]].y + (0.5 * img.rows));
      auto kpt_end =
          cv::Point(blade_array_[i].kpt[kpt_idx[j + 2]].x + (0.5 * img.cols),
                    blade_array_[i].kpt[kpt_idx[j + 2]].y + (0.5 * img.rows));
      cv::line(img, kpt_start, kpt_end, cv::Scalar(0, 255, 0), 4);
    }
    cv::circle(img,
               cv::Point(blade_array_[i].kpt[2].x + (0.5 * img.cols),
                         blade_array_[i].kpt[2].y + (0.5 * img.rows)),
               8, cv::Scalar(255, 0, 0), -1);
  }
}

}  // namespace rm_buff
