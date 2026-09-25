#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <peel/GLib/GLib.h>
#include <peel/Gst/Gst.h>

namespace media_service {

struct PipelineConfig {
  std::string camera_device;
  int video_width;
  int video_height;
  int video_fps;

  std::string mic_device;

  std::string speaker_device;

  std::string signaller_uri;
  bool run_signalling_server;
  bool run_web_server;
  std::string robot_producer_peer_id;
  std::string operator_audio_producer_peer_id;
  bool single_operator;
  bool lan_only;

  int initial_backoff_ms;
  int max_backoff_ms;
  int watchdog_hz;
};

class GstPipelineManager {
public:
  GstPipelineManager();
  ~GstPipelineManager();

  GstPipelineManager(const GstPipelineManager &) = delete;
  GstPipelineManager &operator=(const GstPipelineManager &) = delete;

  bool init(const PipelineConfig &config);
  bool startSendPipeline();
  bool startReceivePipeline();
  void stop();

  bool isRunning() const { return running_.load(); }

private:
  peel::RefPtr<peel::Gst::Pipeline>
  buildPipeline(const std::string &name, const std::string &description);

  void teardownPipeline(peel::RefPtr<peel::Gst::Pipeline> &pipeline);

  bool onBusMessage(peel::Gst::Bus *bus, peel::Gst::Message *message);

  void startMainLoop();
  void stopMainLoop();

  bool isRecvPipelineSource(peel::Gst::Message *message) const;
  void scheduleRecvRestart();
  static gboolean onRecvRestartTimeout(gpointer user_data);

  PipelineConfig config_;

  peel::RefPtr<peel::Gst::Pipeline> send_pipeline_;
  peel::RefPtr<peel::Gst::Pipeline> recv_pipeline_;

  peel::RefPtr<peel::GLib::MainLoop> mainloop_;
  std::thread mainloop_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> recv_restart_pending_{false};
  int recv_backoff_ms_ = 0;
  int recv_retry_count_ = 0;
  bool gst_initialised_ = false;
};

} // namespace media_service
