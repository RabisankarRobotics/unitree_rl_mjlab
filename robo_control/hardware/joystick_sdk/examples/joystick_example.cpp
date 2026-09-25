#include "joystick_sdk/joystick.hpp"

#include <iomanip>
#include <iostream>
#include <unistd.h>

int main() {
  try {
    joystick_sdk::Joystick joystick;

    std::cout << "Joystick connected: " << joystick.getDeviceName()
              << std::endl;
    std::cout << "Device path: " << joystick.getDevicePath() << std::endl;
    std::cout << "\nReading joystick state (Ctrl+C to exit)...\n" << std::endl;

    while (true) {
      joystick_sdk::JoystickState state = joystick.getState();

      std::cout << "\r";
      std::cout << "X:" << std::fixed << std::setprecision(2) << std::setw(6)
                << state.x << " ";
      std::cout << "Y:" << std::fixed << std::setprecision(2) << std::setw(6)
                << state.y << " ";
      std::cout << "RX:" << std::fixed << std::setprecision(2) << std::setw(6)
                << state.rx << " ";
      std::cout << "RY:" << std::fixed << std::setprecision(2) << std::setw(6)
                << state.ry << " ";
      std::cout << "LT:" << std::fixed << std::setprecision(2) << std::setw(6)
                << state.lt << " ";
      std::cout << "RT:" << std::fixed << std::setprecision(2) << std::setw(6)
                << state.rt << " ";
      std::cout << "Buttons:[";
      auto pressed = joystick.getPressedButtons();
      for (const auto &button : pressed) {
        switch (button) {
        case joystick_sdk::ButtonName::A:
          std::cout << "A ";
          break;
        case joystick_sdk::ButtonName::B:
          std::cout << "B ";
          break;
        case joystick_sdk::ButtonName::X:
          std::cout << "X ";
          break;
        case joystick_sdk::ButtonName::Y:
          std::cout << "Y ";
          break;
        case joystick_sdk::ButtonName::LB:
          std::cout << "LB ";
          break;
        case joystick_sdk::ButtonName::RB:
          std::cout << "RB ";
          break;
        case joystick_sdk::ButtonName::BACK:
          std::cout << "BACK ";
          break;
        case joystick_sdk::ButtonName::START:
          std::cout << "START ";
          break;
        case joystick_sdk::ButtonName::GUIDE:
          std::cout << "GUIDE ";
          break;
        case joystick_sdk::ButtonName::LS:
          std::cout << "LS ";
          break;
        case joystick_sdk::ButtonName::RS:
          std::cout << "RS ";
          break;
        }
      }
      std::cout << "] ";
      std::cout << "Hat:[" << (int)state.hat_x << "," << (int)state.hat_y
                << "]";
      std::cout << std::flush;

      usleep(10000);
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
