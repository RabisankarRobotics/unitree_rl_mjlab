#include "mode_service/mode_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  return transport::run_service<mode_service::ModeService>("mode_service", argc,
                                                           argv);
}
