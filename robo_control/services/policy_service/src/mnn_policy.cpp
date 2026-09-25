#include "policy_service/mnn_policy.hpp"

#include <spdlog/spdlog.h>
#include <stdexcept>

namespace policy_service {

MnnPolicy::MnnPolicy(const std::string &model_path, int input_size,
                     int output_size)
    : input_size_(input_size), output_size_(output_size) {

  interpreter_.reset(MNN::Interpreter::createFromFile(model_path.c_str()));
  if (!interpreter_) {
    throw std::runtime_error("Failed to create MNN interpreter from: " +
                             model_path);
  }

  MNN::ScheduleConfig schedule_config;
  schedule_config.type = MNN_FORWARD_CPU;
  schedule_config.numThread = 1;

  MNN::BackendConfig backend_config;
  backend_config.precision = MNN::BackendConfig::Precision_High;
  backend_config.memory = MNN::BackendConfig::Memory_Normal;
  backend_config.power = MNN::BackendConfig::Power_Normal;
  schedule_config.backendConfig = &backend_config;

  session_ = interpreter_->createSession(schedule_config);
  if (!session_) {
    throw std::runtime_error("Failed to create MNN session");
  }

  MNN::Tensor *input_tensor = interpreter_->getSessionInput(session_, nullptr);
  if (!input_tensor) {
    throw std::runtime_error("MNN model must have at least one input tensor");
  }

  std::vector<int> input_shape = {1, input_size_};
  interpreter_->resizeTensor(input_tensor, input_shape);
  interpreter_->resizeSession(session_);

  input_tensor_host_.reset(
      new MNN::Tensor(input_tensor, input_tensor->getDimensionType()));

  spdlog::info("MNN policy loaded: input_size={}, output_size={}", input_size_,
               output_size_);
}

MnnPolicy::~MnnPolicy() {
  if (interpreter_ && session_) {
    interpreter_->releaseSession(session_);
    session_ = nullptr;
  }
  input_tensor_host_.reset();
  interpreter_.reset();
}

MnnPolicy::MnnPolicy(MnnPolicy &&other) noexcept
    : input_size_(other.input_size_), output_size_(other.output_size_),
      interpreter_(std::move(other.interpreter_)), session_(other.session_),
      input_tensor_host_(std::move(other.input_tensor_host_)) {
  other.session_ = nullptr;
}

MnnPolicy &MnnPolicy::operator=(MnnPolicy &&other) noexcept {
  if (this != &other) {
    if (interpreter_ && session_) {
      interpreter_->releaseSession(session_);
    }
    input_size_ = other.input_size_;
    output_size_ = other.output_size_;
    interpreter_ = std::move(other.interpreter_);
    session_ = other.session_;
    input_tensor_host_ = std::move(other.input_tensor_host_);
    other.session_ = nullptr;
  }
  return *this;
}

std::vector<float> MnnPolicy::infer(const std::vector<float> &input) {
  if (!interpreter_ || !session_) {
    throw std::runtime_error("MNN model not loaded");
  }

  if (static_cast<int>(input.size()) != input_size_) {
    throw std::runtime_error("Input size mismatch: expected " +
                             std::to_string(input_size_) + ", got " +
                             std::to_string(input.size()));
  }

  MNN::Tensor *input_tensor = interpreter_->getSessionInput(session_, nullptr);
  if (!input_tensor) {
    throw std::runtime_error("MNN input tensor not available");
  }

  // Ensure host tensor is properly sized
  if (!input_tensor_host_ ||
      input_tensor_host_->elementSize() != static_cast<int>(input.size())) {
    input_tensor_host_.reset(
        new MNN::Tensor(input_tensor, input_tensor->getDimensionType()));
  }

  std::copy(input.begin(), input.end(), input_tensor_host_->host<float>());
  input_tensor->copyFromHostTensor(input_tensor_host_.get());

  interpreter_->runSession(session_);

  MNN::Tensor *output_tensor =
      interpreter_->getSessionOutput(session_, nullptr);
  if (!output_tensor) {
    throw std::runtime_error("MNN model returned no outputs");
  }

  MNN::Tensor output_host(output_tensor, output_tensor->getDimensionType());
  output_tensor->copyToHostTensor(&output_host);

  if (output_host.elementSize() < output_size_) {
    throw std::runtime_error("MNN output has fewer than expected elements");
  }

  std::vector<float> actions(output_size_);
  std::copy_n(output_host.host<float>(), output_size_, actions.begin());
  return actions;
}

} // namespace policy_service
