#include "buff_detector/detector.hpp"

namespace rm_buff {
Detector::Detector(const std::string model_path) : model_path_(model_path) {
  core_ = ov::Core();
  model_ = core_.read_model(model_path_);

  // convert layout from [1, 18, 8400] to [1, 8400, 18]
  ov::preprocess::PrePostProcessor ppp(model_);
  ppp.output().postprocess().convert_layout({0, 2, 1});
  model_ = ppp.build();

  compiled_model_ = core_.compile_model(model_, "CPU");
  infer_request_ = compiled_model_.create_infer_request();
  auto input_info = compiled_model_.input();
  // std::cout << "input_info: " << input_info << std::endl;
  input_tensor_ = infer_request_.get_input_tensor(0);
}

// Detector::Detector() {}

// Detector::~Detector() {}

buff_interfaces::msg::BladeArray Detector::Detect(cv::Mat &src_img) {
  cv::Mat img;
  // cv::cvtColor(src_img, img, cv::COLOR_BGR2RGB);
  // cv::Mat img_input;
  cv::resize(src_img, img, cv::Size(IMG_SIZE, IMG_SIZE));
  // cv::Mat img_input = letterbox(img, IMG_SIZE, IMG_SIZE);
  // img_input.convertTo(img_input, CV_32FC3, 1.0 / 255);
  // cv::imshow("img_input", img_input);
  // cv::waitKey(1);

  // std::vector<float> img_data;
  // img_data.assign((float *)img_input.datastart, (float *)img_input.dataend);

  auto data = input_tensor_.data<float>();

  for (int h = 0; h < IMG_SIZE; h++) {
    for (int w = 0; w < IMG_SIZE; w++) {
      for (int c = 0; c < 3; c++) {
        int out_index = c * IMG_SIZE * IMG_SIZE + h * IMG_SIZE + w;
        data[out_index] = float(img.at<cv::Vec3b>(h, w)[c]) / 255.0f;
      }
    }
  }

  // std::copy(img_data.begin(), img_data.end(), data);
  infer_request_.infer();
  // infer_request_.start_async();
  // infer_request_.wait();
  auto output = infer_request_.get_output_tensor(0);
  // std::cout << "output: " << output.get_shape() << std::endl;

  buff_interfaces::msg::BladeArray blade_array =
      non_max_suppression(output, CONF_THRESHOLD, NMS_THRESHOLD, CLS_NUM);

  // std::vector<int> indices;

  // cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD,
  // indices,
  //                   CONF_REMAIN);

  // for (size_t i = 0; i < indices.size(); ++i) {
  //   int idx = indices[i];
  //   std::cout << "idx: " << idx << std::endl;
  // }

  for (size_t i = 0; i < blade_array.blades.size(); ++i) {
    blade_array.blades[i].x /= IMG_SIZE;
    blade_array.blades[i].y /= IMG_SIZE;
    blade_array.blades[i].width /= IMG_SIZE;
    blade_array.blades[i].height /= IMG_SIZE;
    for (size_t j = 0; j < blade_array.blades[i].kpt.size(); ++j) {
      blade_array.blades[i].kpt[j].x /= IMG_SIZE;
      blade_array.blades[i].kpt[j].y /= IMG_SIZE;
    }
  }

  return blade_array;
}

buff_interfaces::msg::BladeArray Detector::non_max_suppression(
    ov::Tensor &output, float conf_thres, float iou_thres, int nc) {
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
        kpt.x = data[offset + 4 + nc + k * 2];
        kpt.y = data[offset + 4 + nc + k * 2 + 1];
        kpts.push_back(kpt);
      }
      kpts_list.push_back(kpts);
    }
  }

  cv::dnn::softNMSBoxes(boxes, confidences, picked_useless, conf_thres,
                        iou_thres, picked);

  buff_interfaces::msg::BladeArray output_list;

  for (size_t i = 0; i < picked.size(); ++i) {
    buff_interfaces::msg::Blade blade;
    int idx = picked[i];
    blade.x = boxes[idx].x;
    blade.y = boxes[idx].y;
    blade.width = boxes[idx].width;
    blade.height = boxes[idx].height;
    blade.label = classIds[idx];
    blade.prob = confidences[idx];
    blade.kpt = kpts_list[idx];
    // std::cout << "idx: " << idx << std::endl;
    // std::cout << "x: " << boxes[idx].x << " y: " << boxes[idx].y
    //           << " w: " << boxes[idx].width << " h: " << boxes[idx].height
    //           << std::endl;
    // std::cout << "conf: " << confidences[idx] << std::endl;
    // for (int k = 0; k < 10; k++) {
    //   std::cout << data[idx * ml + 4 + k] << " ";
    // }
    output_list.blades.push_back(blade);
  }

  return output_list;
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

}  // namespace rm_buff
