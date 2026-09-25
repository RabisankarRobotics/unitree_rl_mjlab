#pragma once

#include <cstdint>
#include <string>

#include <peel/Gio/Subprocess.h>
#include <peel/RefPtr.h>

namespace media_service {

class SignallingServer {
public:
  SignallingServer();
  ~SignallingServer();

  SignallingServer(const SignallingServer &) = delete;
  SignallingServer &operator=(const SignallingServer &) = delete;

  bool start(const std::string &signaller_uri);
  void stop();
  bool isRunning() const;

  static bool parseUri(const std::string &uri, std::string &host,
                       uint16_t &port);

private:
  bool waitForReady(int timeout_ms = 5000, int interval_ms = 100);

  peel::RefPtr<peel::Gio::Subprocess> process_;
  std::string host_;
  uint16_t port_ = 0;

  static constexpr const char *kBinary = "gst-webrtc-signalling-server";
};

} // namespace media_service
