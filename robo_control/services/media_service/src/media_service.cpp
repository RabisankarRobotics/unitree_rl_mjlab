#include "media_service/media_service.hpp"

#include <common/config_loader.hpp>
#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

namespace media_service {

MediaService::MediaService() = default;

MediaService::~MediaService() { stop(); }

void MediaService::loadParameters() {
  auto node = transport::NodeContext::instance().node();

  auto config_path = node->declare_parameter<std::string>("config_path");
  YAML::Node config = YAML::LoadFile(config_path);

  auto cam = config["camera"];
  config_.camera_device = common::get_config_value<std::string>(cam, "device");
  config_.video_width = common::get_config_value<int>(cam, "width");
  config_.video_height = common::get_config_value<int>(cam, "height");
  config_.video_fps = common::get_config_value<int>(cam, "fps");

  config_.mic_device = common::get_config_value<std::string>(
      config["microphone"], "device");

  config_.speaker_device = common::get_config_value<std::string>(
      config["speaker"], "device");

  auto wrtc = config["webrtc"];
  config_.signaller_uri =
      common::get_config_value<std::string>(wrtc, "signaller_uri");
  config_.run_signalling_server =
      common::get_config_value<bool>(wrtc, "run_signalling_server");
  config_.run_web_server =
      common::get_config_value<bool>(wrtc, "run_web_server");
  config_.robot_producer_peer_id =
      common::get_config_value<std::string>(wrtc, "robot_producer_peer_id");
  config_.operator_audio_producer_peer_id =
      common::get_config_value<std::string>(
          wrtc, "operator_audio_producer_peer_id");
  config_.single_operator =
      common::get_config_value<bool>(wrtc, "single_operator");
  config_.lan_only = common::get_config_value<bool>(wrtc, "lan_only");

  auto rst = config["restart"];
  config_.initial_backoff_ms =
      common::get_config_value<int>(rst, "initial_backoff_ms");
  config_.max_backoff_ms =
      common::get_config_value<int>(rst, "max_backoff_ms");
  config_.watchdog_hz = common::get_config_value<int>(rst, "watchdog_hz");
}

bool MediaService::init() {
  loadParameters();

  spdlog::info("MediaService config: camera={}, video={}x{}@{}fps, "
               "mic={}, spk={}, signaller_uri={}",
               config_.camera_device, config_.video_width, config_.video_height,
               config_.video_fps, config_.mic_device, config_.speaker_device,
               config_.signaller_uri);

  if (config_.run_signalling_server) {
    signalling_server_ = std::make_unique<SignallingServer>();
    if (!signalling_server_->start(config_.signaller_uri)) {
      spdlog::error("Failed to start signalling server");
      return false;
    }
  }

  pipeline_manager_ = std::make_unique<GstPipelineManager>();
  if (!pipeline_manager_->init(config_)) {
    spdlog::error("Failed to initialise GStreamer pipeline manager");
    return false;
  }

  spdlog::info("MediaService initialised");
  return true;
}

void MediaService::start() {
  if (!pipeline_manager_) {
    spdlog::error("MediaService not initialised, cannot start");
    return;
  }

  if (!pipeline_manager_->startSendPipeline()) {
    spdlog::error("Failed to start send pipeline");
  }

  if (!pipeline_manager_->startReceivePipeline()) {
    spdlog::warn("Failed to start receive pipeline "
                 "(will retry when a remote peer connects)");
  }

  spdlog::info("MediaService started");
}

void MediaService::stop() {
  if (pipeline_manager_) {
    pipeline_manager_->stop();
    pipeline_manager_.reset();
  }

  if (signalling_server_) {
    signalling_server_->stop();
    signalling_server_.reset();
  }

  spdlog::info("MediaService stopped");
}

} // namespace media_service
