#include "media_service/gst_pipeline_manager.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include <gst/gst.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

using namespace peel;

namespace media_service {

GstPipelineManager::GstPipelineManager() = default;

GstPipelineManager::~GstPipelineManager() { stop(); }

bool GstPipelineManager::init(const PipelineConfig &config) {
  config_ = config;
  recv_backoff_ms_ = config_.initial_backoff_ms;

  if (!gst_initialised_) {
    UniquePtr<GLib::Error> error;
    if (!Gst::init_check(nullptr, nullptr, &error)) {
      spdlog::error("GStreamer init failed: {}",
                    error ? error->message : "unknown");
      return false;
    }
    gst_initialised_ = true;

    String ver = Gst::version_string();
    spdlog::info("GStreamer initialised ({})", ver.c_str());
  }

  return true;
}

RefPtr<Gst::Pipeline>
GstPipelineManager::buildPipeline(const std::string &name,
                                  const std::string &description) {
  spdlog::debug("Building pipeline '{}': {}", name, description);

  UniquePtr<GLib::Error> error;
  auto pipeline = Gst::parse_launch(description.c_str(), &error)
                      .ref_sink()
                      .cast<Gst::Pipeline>();

  if (!pipeline || error) {
    spdlog::error("Failed to create pipeline '{}': {}", name,
                  error ? error->message : "unknown parse error");
    return {};
  }

  auto bus = pipeline->get_bus();
  if (bus) {
    bus->add_watch_full(G_PRIORITY_DEFAULT,
                        [this](Gst::Bus *b, Gst::Message *m) {
                          return onBusMessage(b, m);
                        });
  }

  spdlog::debug("Pipeline '{}' created successfully", name);
  return pipeline;
}

void GstPipelineManager::teardownPipeline(
    RefPtr<Gst::Pipeline> &pipeline) {
  if (!pipeline)
    return;

  pipeline->set_state(Gst::State::NULL_);

  auto bus = pipeline->get_bus();
  if (bus) {
    bus->remove_watch();
  }

  pipeline = {};
}

bool GstPipelineManager::startSendPipeline() {
  std::string desc = fmt::format(
      "v4l2src device={camera} ! "
      "videoconvert ! "
      "videoscale ! "
      "video/x-raw,width={w},height={h},framerate={fps}/1 ! "
      "queue max-size-buffers=3 leaky=downstream ! "
      "webrtcsink name=ws "
      "signaller::uri={uri} "
      "alsasrc device={mic} ! "
      "audioconvert ! "
      "audioresample ! "
      "queue max-size-buffers=10 leaky=downstream ! "
      "ws. ",
      fmt::arg("camera", config_.camera_device),
      fmt::arg("w", config_.video_width),
      fmt::arg("h", config_.video_height),
      fmt::arg("fps", config_.video_fps),
      fmt::arg("uri", config_.signaller_uri),
      fmt::arg("mic", config_.mic_device));

  send_pipeline_ = buildPipeline("send", desc);
  if (!send_pipeline_)
    return false;

  startMainLoop();

  auto ret = send_pipeline_->set_state(Gst::State::PLAYING);
  if (ret == Gst::StateChangeReturn::FAILURE) {
    spdlog::error("Send pipeline failed to transition to PLAYING");
    teardownPipeline(send_pipeline_);
    return false;
  }

  running_.store(true);
  spdlog::info("Send pipeline PLAYING (signaller={}, "
               "producer-peer-id={})",
               config_.signaller_uri, config_.robot_producer_peer_id);
  return true;
}

bool GstPipelineManager::startReceivePipeline() {
  std::string desc = fmt::format(
      "webrtcsrc name=src "
      "signaller::uri={uri} "
      "signaller::producer-peer-id={peer_id} ! "
      "decodebin3 ! "
      "audioconvert ! "
      "audioresample ! "
      "alsasink device={spk}",
      fmt::arg("uri", config_.signaller_uri),
      fmt::arg("peer_id", config_.operator_audio_producer_peer_id),
      fmt::arg("spk", config_.speaker_device));

  recv_pipeline_ = buildPipeline("recv", desc);
  if (!recv_pipeline_)
    return false;

  startMainLoop();

  auto ret = recv_pipeline_->set_state(Gst::State::PLAYING);
  if (ret == Gst::StateChangeReturn::FAILURE) {
    spdlog::error("Receive pipeline failed to transition to PLAYING");
    teardownPipeline(recv_pipeline_);
    return false;
  }

  if (recv_retry_count_ == 0) {
    spdlog::info("Receive pipeline PLAYING (connecting to signaller at {}, "
                 "producer-peer-id={})",
                 config_.signaller_uri,
                 config_.operator_audio_producer_peer_id);
  }
  return true;
}

void GstPipelineManager::stop() {
  running_.store(false);
  recv_restart_pending_.store(false);

  teardownPipeline(send_pipeline_);
  teardownPipeline(recv_pipeline_);

  stopMainLoop();

  spdlog::info("GstPipelineManager stopped");
}

void GstPipelineManager::startMainLoop() {
  if (mainloop_)
    return;

  mainloop_ = GLib::MainLoop::create(nullptr, false);

  mainloop_thread_ = std::thread([this]() {
    spdlog::debug("GMainLoop thread started");
    mainloop_->run();
    spdlog::debug("GMainLoop thread exiting");
  });
}

void GstPipelineManager::stopMainLoop() {
  if (mainloop_) {
    mainloop_->quit();
  }

  if (mainloop_thread_.joinable()) {
    mainloop_thread_.join();
  }

  mainloop_ = {};
}

bool GstPipelineManager::onBusMessage(Gst::Bus * /*bus*/,
                                      Gst::Message *message) {
  switch (message->type) {
  case Gst::Message::Type::ERROR_: {
    UniquePtr<GLib::Error> err;
    String debug_info;
    message->parse_error(&err, &debug_info);

    if (isRecvPipelineSource(message)) {
      if (recv_retry_count_ == 0) {
        spdlog::info("Waiting for producer '{}' (retrying every {}–{}ms)",
                      config_.operator_audio_producer_peer_id,
                      config_.initial_backoff_ms, config_.max_backoff_ms);
      } else {
        spdlog::debug("Receive pipeline: producer '{}' not available yet, "
                      "retry #{} in {}ms",
                      config_.operator_audio_producer_peer_id,
                      recv_retry_count_, recv_backoff_ms_);
      }
      ++recv_retry_count_;
      scheduleRecvRestart();
    } else {
      spdlog::error("GStreamer error from {}: {}",
                    message->src->get_name().c_str(),
                    err ? err->message : "unknown");
      if (debug_info) {
        spdlog::debug("  Debug info: {}", debug_info.c_str());
      }
    }
    break;
  }

  case Gst::Message::Type::WARNING: {
    UniquePtr<GLib::Error> err;
    String debug_info;
    message->parse_warning(&err, &debug_info);

    spdlog::warn("GStreamer warning from {}: {}",
                 message->src->get_name().c_str(),
                 err ? err->message : "unknown");
    break;
  }

  case Gst::Message::Type::STATE_CHANGED: {
    if (message->src == send_pipeline_ ||
        message->src == recv_pipeline_) {
      Gst::State old_state, new_state, pending;
      message->parse_state_changed(&old_state, &new_state, &pending);

      if (message->src == recv_pipeline_ && recv_retry_count_ > 0) {
        spdlog::debug("Pipeline '{}' state: {} -> {}",
                      message->src->get_name().c_str(),
                      Gst::Element::state_get_name(old_state),
                      Gst::Element::state_get_name(new_state));
        if (new_state == Gst::State::PLAYING) {
          spdlog::info("Producer '{}' connected, receive pipeline active",
                       config_.operator_audio_producer_peer_id);
          recv_retry_count_ = 0;
          recv_backoff_ms_ = config_.initial_backoff_ms;
        }
      } else {
        spdlog::info("Pipeline '{}' state: {} -> {}",
                     message->src->get_name().c_str(),
                     Gst::Element::state_get_name(old_state),
                     Gst::Element::state_get_name(new_state));
      }
    }
    break;
  }

  case Gst::Message::Type::EOS: {
    spdlog::info("End-of-stream on '{}'",
                 message->src->get_name().c_str());
    break;
  }

  case Gst::Message::Type::LATENCY: {
    if (send_pipeline_) {
      send_pipeline_->recalculate_latency();
    }
    if (recv_pipeline_) {
      recv_pipeline_->recalculate_latency();
    }
    break;
  }

  default:
    break;
  }

  return true;
}

bool GstPipelineManager::isRecvPipelineSource(Gst::Message *message) const {
  if (!recv_pipeline_ || !message->src)
    return false;

  auto *src = reinterpret_cast<GstObject *>(message->src);
  auto *pipeline =
      reinterpret_cast<GstObject *>(static_cast<Gst::Pipeline *>(recv_pipeline_));

  return src == pipeline || gst_object_has_as_ancestor(src, pipeline);
}

void GstPipelineManager::scheduleRecvRestart() {
  if (recv_restart_pending_.exchange(true))
    return;

  g_timeout_add(recv_backoff_ms_, &GstPipelineManager::onRecvRestartTimeout,
                this);
  recv_backoff_ms_ = std::min(recv_backoff_ms_ * 2, config_.max_backoff_ms);
}

gboolean GstPipelineManager::onRecvRestartTimeout(gpointer user_data) {
  auto *self = static_cast<GstPipelineManager *>(user_data);
  self->recv_restart_pending_.store(false);

  if (!self->running_.load())
    return G_SOURCE_REMOVE;

  self->teardownPipeline(self->recv_pipeline_);
  self->startReceivePipeline();

  return G_SOURCE_REMOVE;
}

} // namespace media_service
