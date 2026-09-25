#include <imu_sdk/bno08x_imu.hpp>
#include <iostream>
#include <string>
#include <unistd.h>

int main(int argc, char *argv[]) {
  std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
  imu_sdk::BNO08xIMU imu(port);

  imu.enableFeature(imu_sdk::BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR);
  imu.enableFeature(imu_sdk::BNO_REPORT_GYROSCOPE_CALIBRATED);

  while (true) {
    auto quat = imu.gyroIntegratedRotationVector();
    auto gyro = imu.gyroscopeCalibrated();

    std::cout << "Quat: [" << quat.real << ", " << quat.i << ", " << quat.j
              << ", " << quat.k << "]" << std::endl;
    std::cout << "Gyro: [" << gyro.x << ", " << gyro.y << ", " << gyro.z << "]"
              << std::endl;
    std::cout << std::endl;

    usleep(100000);
  }

  return 0;
}
