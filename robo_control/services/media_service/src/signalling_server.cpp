#include "media_service/signalling_server.hpp"

#include <chrono>
#include <csignal>
#include <thread>

#include <peel/GLib/Error.h>
#include <peel/Gio/SocketClient.h>
#include <peel/Gio/SubprocessFlags.h>
#include <spdlog/spdlog.h>

using namespace peel;

namespace media_service {

SignallingServer::SignallingServer() = default;

SignallingServer::~SignallingServer() { stop(); }

bool SignallingServer::start(const std::string &signaller_uri) {
  if (process_) {
    spdlog::warn("Signalling server already running");
    return true;
  }

  if (!parseUri(signaller_uri, host_, port_)) {
    spdlog::error("Failed to parse signaller URI: {}", signaller_uri);
    return false;
  }

  std::string port_str = std::to_string(port_);

  spdlog::info("Starting signalling server ({} --host {} --port {})", kBinary,
               host_, port_str);

  const char *const argv[] = {kBinary,         "--host", host_.c_str(),
                              "--port", port_str.c_str(), nullptr};

  UniquePtr<GLib::Error> error;
  process_ = Gio::Subprocess::createv(
      argv,
      Gio::Subprocess::Flags::STDOUT_SILENCE |
          Gio::Subprocess::Flags::STDERR_SILENCE,
      &error);

  if (!process_ || error) {
    spdlog::error("Failed to spawn signalling server: {}",
                  error ? error->message : "unknown");
    return false;
  }

  spdlog::info("Signalling server spawned (pid={})",
               process_->get_identifier());

  if (!waitForReady()) {
    spdlog::error(
        "Signalling server did not become ready on {}:{} within timeout",
        host_, port_);
    stop();
    return false;
  }

  spdlog::info("Signalling server ready on {}:{}", host_, port_);
  return true;
}

void SignallingServer::stop() {
  if (!process_)
    return;

  const char *pid = process_->get_identifier();
  if (pid) {
    spdlog::info("Stopping signalling server (pid={})", pid);
    process_->send_signal(SIGTERM);

    constexpr int kGraceMs = 2000;
    constexpr int kPollMs = 50;
    int elapsed = 0;
    while (elapsed < kGraceMs && process_->get_identifier() != nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
      elapsed += kPollMs;
    }

    if (process_->get_identifier() != nullptr) {
      spdlog::warn("Signalling server did not exit after SIGTERM, "
                    "forcing exit");
      process_->force_exit();
    }

    UniquePtr<GLib::Error> error;
    process_->wait(nullptr, &error);
    if (error) {
      spdlog::warn("Error waiting for signalling server exit: {}",
                    error->message);
    }
  }

  process_ = {};
  spdlog::info("Signalling server stopped");
}

bool SignallingServer::isRunning() const {
  return process_ && process_->get_identifier() != nullptr;
}

bool SignallingServer::parseUri(const std::string &uri, std::string &host,
                                uint16_t &port) {
  std::string remainder;
  if (uri.rfind("ws://", 0) == 0) {
    remainder = uri.substr(5);
  } else if (uri.rfind("wss://", 0) == 0) {
    remainder = uri.substr(6);
  } else {
    return false;
  }

  if (!remainder.empty() && remainder.back() == '/')
    remainder.pop_back();

  auto colon = remainder.rfind(':');
  if (colon == std::string::npos || colon == 0)
    return false;

  host = remainder.substr(0, colon);
  std::string port_str = remainder.substr(colon + 1);

  try {
    int p = std::stoi(port_str);
    if (p <= 0 || p > 65535)
      return false;
    port = static_cast<uint16_t>(p);
  } catch (...) {
    return false;
  }

  return true;
}

bool SignallingServer::waitForReady(int timeout_ms, int interval_ms) {
  auto client = Gio::SocketClient::create();
  std::string host_and_port = host_ + ":" + std::to_string(port_);

  int elapsed = 0;
  while (elapsed < timeout_ms) {
    UniquePtr<GLib::Error> error;
    auto conn =
        client->connect_to_host(host_and_port.c_str(), port_, nullptr, &error);
    if (conn)
      return true;

    if (process_->get_identifier() == nullptr) {
      spdlog::error("Signalling server process exited prematurely");
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    elapsed += interval_ms;
  }

  return false;
}

} // namespace media_service
