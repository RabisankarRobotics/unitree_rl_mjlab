#include "imu_service/imu_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  transport::ServiceOptions options;
  options.lock_memory = true;
  return transport::run_service<imu_service::IMUService>("imu_service", argc,
                                                         argv, options);
}
