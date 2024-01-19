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
}

// Detector::Detector() {}

// Detector::~Detector() {}

buff_interfaces::msg::BladeArray Detector::Detect(cv::Mat &src_img) {
  cv::Mat img;

  cv::resize(src_img, img, cv::Size(IMG_SIZE, IMG_SIZE));
  // img = letterbox(src_img, IMG_SIZE, IMG_SIZE);

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
  // for (int h = 0; h < IMG_SIZE; h++) {
  //   for (int w = 0; w < IMG_SIZE; w++) {
  //     for (int c = 0; c < 3; c++) {
  //       int out_index = c * IMG_SIZE * IMG_SIZE + h * IMG_SIZE + w;
  //       data[out_index] = float(img.at<cv::Vec3f>(h, w)[c]);
  //     }
  //   }
  // }

  infer_request_.infer();
  // infer_request_.start_async();
  // infer_request_.wait();
  auto output = infer_request_.get_output_tensor(0);

  non_max_suppression(output, CONF_THRESHOLD, NMS_THRESHOLD, CLS_NUM);

  return blade_array_;
}

void Detector::non_max_suppression(ov::Tensor &output, float conf_thres,
                                   float iou_thres, int nc) {
  auto data = output.data<float>();

  int bs = output.get_shape()[0];  // batch size
  int nm = output.get_shape()[1];  // number of masks
  int ml = output.get_shape()[2];  // mask length

  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<geometry_msgs::msg::Point>> kpts_list;
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
      classIds.push_back(classId);
      confidences.push_back(conf);
      boxes.push_back(cv::Rect(x, y, w, h));
      std::vector<geometry_msgs::msg::Point> kpts;
      for (int k = 0; k < KPT_NUM; k++) {
        geometry_msgs::msg::Point kpt;
        kpt.x = data[offset + 4 + nc + k * 2] / IMG_SIZE;
        kpt.y = data[offset + 4 + nc + k * 2 + 1] / IMG_SIZE;
        kpts.push_back(kpt);
      }
      kpts_list.push_back(kpts);
    }
  }

  cv::dnn::softNMSBoxes(boxes, confidences, picked_useless, conf_thres,
                        iou_thres, picked);

  blade_array_.blades.clear();
  for (size_t i = 0; i < picked.size(); ++i) {
    buff_interfaces::msg::Blade blade;
    int idx = picked[i];
    blade.x = float(boxes[idx].x) / IMG_SIZE;
    blade.y = float(boxes[idx].y) / IMG_SIZE;
    blade.width = float(boxes[idx].width) / IMG_SIZE;
    blade.height = float(boxes[idx].height) / IMG_SIZE;
    blade.label = classIds[idx];
    blade.prob = confidences[idx];
    blade.kpt = kpts_list[idx];
    blade_array_.blades.push_back(blade);
  }

  return;
}

cv::Mat Detector::letterbox(cv::Mat &src, int h, int w) {
  int in_w = src.cols;  // width
  int in_h = src.rows;  // height
  int tar_w = w;
  int tar_h = h;
  float r = std::min(float(tar_h) / in_h, float(tar_w) / in_w);
  int inside_w = round(in_w * r);
  int inside_h = round(in_h * r);
  int padd_w = tar_w - inside_w;
  int padd_h = tar_h - inside_h;

  cv::Mat resize_img;

  cv::resize(src, resize_img, cv::Size(inside_w, inside_h));

  padd_w = padd_w / 2;
  padd_h = padd_h / 2;

  int top = int(round(padd_h - 0.1));
  int bottom = int(round(padd_h + 0.1));
  int left = int(round(padd_w - 0.1));
  int right = int(round(padd_w + 0.1));
  cv::copyMakeBorder(resize_img, resize_img, top, bottom, left, right, 0,
                     cv::Scalar(114, 114, 114));

  return resize_img;
}

void Detector::draw_blade(cv::Mat &img) {
  for (size_t i = 0; i < blade_array_.blades.size(); ++i) {
    int kpt_idx[4] = {0, 1, 3, 4};
    for (int j = 0; j < 2; j++) {
      auto kpt_start =
          cv::Point(int(blade_array_.blades[i].kpt[kpt_idx[j]].x * img.cols),
                    int(blade_array_.blades[i].kpt[kpt_idx[j]].y * img.rows));
      auto kpt_end = cv::Point(
          int(blade_array_.blades[i].kpt[kpt_idx[j + 2]].x * img.cols),
          int(blade_array_.blades[i].kpt[kpt_idx[j + 2]].y * img.rows));
      cv::line(img, kpt_start, kpt_end, cv::Scalar(0, 255, 0), 4);
    }
    cv::circle(img,
               cv::Point(blade_array_.blades[i].kpt[2].x * img.cols,
                         blade_array_.blades[i].kpt[2].y * img.rows),
               8, cv::Scalar(255, 0, 0), -1);
  }
}

}  // namespace rm_buff
