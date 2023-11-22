#ifndef BUFF_DETECTOR__DETECTOR_HPP_
#define BUFF_DETECTOR__DETECTOR_HPP_

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "openvino/openvino.hpp"

namespace rm_buff {
class Detector {
 public:
  explicit Detector();
  explicit Detector(const std::string model_path);
  ~Detector();

  std::vector<float> Detect(const cv::Mat& image);

 private:
  std::string model_path_;
  std::string bin_path_;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Tensor input_tensor_;
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__DETECTOR_HPP_
