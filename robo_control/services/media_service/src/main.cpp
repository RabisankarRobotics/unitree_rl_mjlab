#include "media_service/media_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  transport::ServiceOptions options;
  options.lock_memory = false; // No real-time requirements for media streaming
  return transport::run_service<media_service::MediaService>("media_service",
                                                             argc, argv,
                                                             options);
}
