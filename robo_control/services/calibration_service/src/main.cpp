#include "calibration_service/calibration_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  return transport::run_service<calibration_service::CalibrationService>(
      "calibration_service", argc, argv);
}
