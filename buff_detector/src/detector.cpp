#include "buff_detector/detector.hpp"

namespace rm_buff {
Detector::Detector(const std::string model_path) : model_path_(model_path) {
  core_ = ov::Core();
  model_ = core_.read_model(model_path_);
  compiled_model_ = core_.compile_model(model_, "CPU");
  infer_request_ = compiled_model_.create_infer_request();
  auto input_info = compiled_model_.input();
  // log input_info
  std::cout << "input_info: " << input_info << std::endl;
  input_tensor_ = infer_request_.get_input_tensor(0);
}

Detector::Detector() {}

Detector::~Detector() {}

std::vector<float> Detector::Detect(const cv::Mat &image) {
  auto input_data = input_tensor_.data<float>();
  cv::Mat resize_image;
  int height = 640;
  int width = 640;
  cv::resize(image, resize_image, cv::Size(height, width));
  cv::cvtColor(resize_image, resize_image, cv::COLOR_BGR2RGB);
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      for (int c = 0; c < 3; c++) {
        int out_index = c * height * width + h * width + w;
        input_data[out_index] =
            float(resize_image.at<cv::Vec3b>(h, w)[c]) / 255.0f;
      }
    }
  }

  infer_request_.infer();
  ov::Tensor output_tensor = infer_request_.get_output_tensor();
  std::vector<float> result;
  for (int i = 0; i < 100; i++) {
    result.push_back(output_tensor.data<float>()[i]);
  }
  return result;
}

}  // namespace rm_buff
