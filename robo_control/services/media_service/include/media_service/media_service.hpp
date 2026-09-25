#pragma once

#include <memory>

#include "media_service/gst_pipeline_manager.hpp"
#include "media_service/signalling_server.hpp"

namespace media_service {

class MediaService {
public:
  MediaService();
  ~MediaService();

  MediaService(const MediaService &) = delete;
  MediaService &operator=(const MediaService &) = delete;

  bool init();
  void start();
  void stop();

private:
  void loadParameters();

  PipelineConfig config_;
  std::unique_ptr<SignallingServer> signalling_server_;
  std::unique_ptr<GstPipelineManager> pipeline_manager_;
};

} // namespace media_service
