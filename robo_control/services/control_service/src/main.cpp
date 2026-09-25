#include "control_service/robot_controller.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  transport::ServiceOptions options;
  options.lock_memory = true;
  return transport::run_service<control_service::RobotController>(
      "control_service", argc, argv, options);
}
