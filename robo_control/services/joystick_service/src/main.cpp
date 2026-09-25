#include "joystick_service/joystick_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  return transport::run_service<joystick_service::JoystickService>(
      "joystick_service", argc, argv);
}
