#pragma once

#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <memory>
#include <string>
#include <vector>

namespace policy_service {

class MnnPolicy {
public:
  MnnPolicy(const std::string &model_path, int input_size, int output_size);
  ~MnnPolicy();

  // Non-copyable
  MnnPolicy(const MnnPolicy &) = delete;
  MnnPolicy &operator=(const MnnPolicy &) = delete;

  // Movable
  MnnPolicy(MnnPolicy &&) noexcept;
  MnnPolicy &operator=(MnnPolicy &&) noexcept;

  std::vector<float> infer(const std::vector<float> &input);

private:
  int input_size_;
  int output_size_;
  std::shared_ptr<MNN::Interpreter> interpreter_;
  MNN::Session *session_{nullptr};
  std::shared_ptr<MNN::Tensor> input_tensor_host_;
};

} // namespace policy_service
