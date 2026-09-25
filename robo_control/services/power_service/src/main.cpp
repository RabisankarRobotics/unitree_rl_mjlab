#include "power_service/power_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  transport::ServiceOptions options;
  options.lock_memory = false;
  return transport::run_service<power_service::PowerService>("power_service",
                                                             argc, argv,
                                                             options);
}
