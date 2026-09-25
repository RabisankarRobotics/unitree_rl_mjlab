#include "imu_sdk/bno08x_imu.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

// SHTP Protocol Constants
#define SHTP_FRAME_START 0x7E
#define SHTP_FRAME_ESCAPE 0x7D
#define SHTP_PROTOCOL_ID 0x01
#define SHTP_HEADER_SIZE 4
#define MAX_PACKET_SIZE 512

// Channels
#define CHANNEL_SHTP_COMMAND 0
#define CHANNEL_EXE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO_RV 5

// Report IDs
#define REPORT_PRODUCT_ID_REQUEST 0xF9
#define REPORT_PRODUCT_ID_RESPONSE 0xF8
#define REPORT_SET_FEATURE 0xFD
#define REPORT_GET_FEATURE_RESPONSE 0xFC
#define REPORT_BASE_TIMESTAMP 0xFB

// Q-point scalars
#define Q_POINT_20_SCALAR (1.0f / (1 << 20))
#define Q_POINT_14_SCALAR (1.0f / (1 << 14))
#define Q_POINT_12_SCALAR (1.0f / (1 << 12))
#define Q_POINT_10_SCALAR (1.0f / (1 << 10))
#define Q_POINT_9_SCALAR (1.0f / (1 << 9))
#define Q_POINT_8_SCALAR (1.0f / (1 << 8))
#define Q_POINT_7_SCALAR (1.0f / (1 << 7))
#define Q_POINT_4_SCALAR (1.0f / (1 << 4))

namespace imu_sdk {
// Helper function to get the expected length of a report based on its ID
static int getReportLength(uint8_t reportId) {
  // Control/command reports (0xF0 and above)
  if (reportId >= 0xF0) {
    switch (reportId) {
    case REPORT_BASE_TIMESTAMP:
      return 5;
    case REPORT_GET_FEATURE_RESPONSE:
      return 17;
    case REPORT_PRODUCT_ID_RESPONSE:
      return 16;
    default:
      return 0;
    }
  }

  // Sensor reports
  switch (reportId) {
  // 10-byte reports (Vector3 with status)
  case BNO_REPORT_ACCELEROMETER:
  case BNO_REPORT_GYROSCOPE_CALIBRATED:
  case BNO_REPORT_MAGNETIC_FIELD_CALIBRATED:
  case BNO_REPORT_LINEAR_ACCELERATION:
  case BNO_REPORT_GRAVITY:
    return 10;

  // Rotation vectors with accuracy (14 bytes)
  case BNO_REPORT_ROTATION_VECTOR:
  case BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR:
  case BNO_REPORT_ARVR_STABILIZED_RV:
    return 14;

  // Rotation vectors without accuracy (12 bytes)
  case BNO_REPORT_GAME_ROTATION_VECTOR:
  case BNO_REPORT_ARVR_STABILIZED_GRV:
    return 12;

  // Gyro integrated RV (14 bytes - different format, no status bytes)
  case BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR:
    return 14;

  // Uncalibrated sensors (16 bytes - includes bias)
  case BNO_REPORT_GYROSCOPE_UNCALIBRATED:
  case BNO_REPORT_MAGNETIC_FIELD_UNCALIBRATED:
    return 16;

  // Raw sensors (16 bytes - includes timestamp)
  case BNO_REPORT_RAW_ACCELEROMETER:
  case BNO_REPORT_RAW_MAGNETOMETER:
    return 16;

  // Raw gyroscope (16 bytes - includes temperature and timestamp)
  case BNO_REPORT_RAW_GYROSCOPE:
    return 16;

  // Environmental sensors (8 bytes)
  case BNO_REPORT_PRESSURE:
  case BNO_REPORT_AMBIENT_LIGHT:
    return 8;

  // Environmental sensors (6 bytes)
  case BNO_REPORT_HUMIDITY:
  case BNO_REPORT_PROXIMITY:
  case BNO_REPORT_TEMPERATURE:
    return 6;

  // Simple detectors (5-6 bytes)
  case BNO_REPORT_TAP_DETECTOR:
    return 5;
  case BNO_REPORT_SIGNIFICANT_MOTION:
  case BNO_REPORT_SHAKE_DETECTOR:
  case BNO_REPORT_FLIP_DETECTOR:
  case BNO_REPORT_PICKUP_DETECTOR:
  case BNO_REPORT_STABILITY_DETECTOR:
  case BNO_REPORT_TILT_DETECTOR:
  case BNO_REPORT_POCKET_DETECTOR:
  case BNO_REPORT_CIRCLE_DETECTOR:
  case BNO_REPORT_HEART_RATE_MONITOR:
    return 6;

  case BNO_REPORT_STABILITY_CLASSIFIER:
  case BNO_REPORT_SLEEP_DETECTOR:
    return 5;

  // Step detector (8 bytes)
  case BNO_REPORT_STEP_DETECTOR:
    return 8;

  // Step counter (12 bytes)
  case BNO_REPORT_STEP_COUNTER:
    return 12;

  // Personal activity classifier (16 bytes)
  case BNO_REPORT_PERSONAL_ACTIVITY_CLASSIFIER:
    return 16;

  // IZRO motion request (6 bytes)
  case BNO_REPORT_IZRO_MOTION_REQUEST:
    return 6;

  default:
    return 0;
  }
}

BNO08xIMU::BNO08xIMU(const std::string &device, int baudrate)
    : uart_fd_(-1), running_(false), pressure_(0), ambientLight_(0),
      humidity_(0), proximity_(0), temperature_(0), tapDetector_(0),
      stepDetectorLatency_(0), significantMotion_(0), stabilityClassifier_(0),
      shakeDetector_(0), flipDetector_(0), pickupDetector_(0),
      stabilityDetector_(0), sleepDetector_(0), tiltDetector_(0),
      pocketDetector_(0), circleDetector_(0), heartRate_(0) {
  memset(sequence_number_, 0, sizeof(sequence_number_));

  // Open UART
  uart_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (uart_fd_ < 0) {
    throw std::runtime_error("Failed to open " + device);
  }

  struct termios tty;
  if (tcgetattr(uart_fd_, &tty) != 0) {
    close(uart_fd_);
    throw std::runtime_error("Error getting UART attributes");
  }

  // Configure UART
  speed_t baud;
  if (baudrate == 3000000)
    baud = B3000000;
  else if (baudrate == 115200)
    baud = B115200;
  else {
    close(uart_fd_);
    throw std::runtime_error("Unsupported baudrate");
  }

  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
    close(uart_fd_);
    throw std::runtime_error("Error setting UART attributes");
  }

  tcflush(uart_fd_, TCIOFLUSH);
  usleep(500000);

  // Initialize sensor with retry logic
  bool initialized = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    try {
      if (softReset() && checkProductId()) {
        initialized = true;
        break;
      }
    } catch (...) {
      // Retry on any exception
    }

    if (attempt < 2) {
      usleep(500000);
    }
  }

  if (!initialized) {
    close(uart_fd_);
    throw std::runtime_error("Failed to initialize sensor after 3 attempts");
  }

  // Start reading thread
  running_ = true;
  read_thread_ = std::thread(&BNO08xIMU::readThreadFunc, this);

  std::cout << "BNO08X initialized on " << device << " at " << baudrate
            << " baud" << std::endl;
}

BNO08xIMU::~BNO08xIMU() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  if (uart_fd_ >= 0) {
    std::set<uint8_t> features_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      features_copy = enabled_features_;
    }

    for (uint8_t feature_id : features_copy) {
      setFeature(feature_id, 0);
    }

    usleep(100000);
    tcflush(uart_fd_, TCIOFLUSH);
    close(uart_fd_);
  }
}

bool BNO08xIMU::sendPacket(uint8_t channel, const uint8_t *data,
                           uint16_t length) {
  if (uart_fd_ < 0)
    return false;

  uint16_t totalLength = length + SHTP_HEADER_SIZE;
  uint8_t header[SHTP_HEADER_SIZE];

  header[0] = totalLength & 0xFF;
  header[1] = (totalLength >> 8) & 0xFF;
  header[2] = channel;
  header[3] = sequence_number_[channel]++;

  auto write_byte = [&](uint8_t b) {
    return write(uart_fd_, &b, 1) == 1;
  };

  if (!write_byte(SHTP_FRAME_START))
    return false;
  usleep(1000);

  if (!write_byte(SHTP_PROTOCOL_ID))
    return false;
  usleep(1000);

  for (int i = 0; i < SHTP_HEADER_SIZE; i++) {
    if (!write_byte(header[i]))
      return false;
    usleep(1000);
  }

  for (int i = 0; i < length; i++) {
    if (!write_byte(data[i]))
      return false;
    usleep(1000);
  }

  if (!write_byte(SHTP_FRAME_START))
    return false;
  usleep(1000);

  return true;
}

bool BNO08xIMU::readPacket(uint8_t *buffer, int &length) {
  if (uart_fd_ < 0)
    return false;

  uint8_t byte;
  int attempts = 0;
  while (true) {
    ssize_t bytes_read = read(uart_fd_, &byte, 1);
    if (bytes_read != 1) {
      attempts++;
      if (attempts > 100)
        return false;
      continue;
    }
    if (byte == SHTP_FRAME_START)
      break;
  }

  if (read(uart_fd_, &byte, 1) != 1)
    return false;

  if (byte == SHTP_FRAME_START) {
    if (read(uart_fd_, &byte, 1) != 1)
      return false;
  }

  if (byte != SHTP_PROTOCOL_ID)
    return false;

  for (int i = 0; i < SHTP_HEADER_SIZE; i++) {
    if (read(uart_fd_, &byte, 1) != 1)
      return false;

    if (byte == SHTP_FRAME_ESCAPE) {
      if (read(uart_fd_, &byte, 1) != 1)
        return false;
      byte ^= 0x20;
    }
    buffer[i] = byte;
  }

  uint16_t packetLength = buffer[0] | (buffer[1] << 8);
  packetLength &= ~0x8000;
  uint16_t dataLength = packetLength - SHTP_HEADER_SIZE;

  if (dataLength > MAX_PACKET_SIZE - SHTP_HEADER_SIZE)
    return false;

  for (int i = 0; i < dataLength; i++) {
    if (read(uart_fd_, &byte, 1) != 1)
      return false;

    if (byte == SHTP_FRAME_ESCAPE) {
      if (read(uart_fd_, &byte, 1) != 1)
        return false;
      byte ^= 0x20;
    }
    buffer[SHTP_HEADER_SIZE + i] = byte;
  }

  if (read(uart_fd_, &byte, 1) != 1 || byte != SHTP_FRAME_START)
    return false;

  length = packetLength;
  return true;
}

void BNO08xIMU::processPacket(const uint8_t *packet, int length) {
  if (length < SHTP_HEADER_SIZE)
    return;

  uint8_t channel = packet[2];
  int dataLength = length - SHTP_HEADER_SIZE;

  if (channel == CHANNEL_CONTROL && dataLength > 0) {
    uint8_t reportId = packet[4];
    if (reportId == REPORT_GET_FEATURE_RESPONSE && dataLength >= 17) {
      uint8_t featureReportId = packet[5];
      std::lock_guard<std::mutex> lock(data_mutex_);
      enabled_features_.insert(featureReportId);
    }
  } else if (channel == CHANNEL_GYRO_RV && dataLength >= 14) {
    // Gyro Integrated RV comes on its own channel with NO header bytes
    // The data is raw: i(2) + j(2) + k(2) + real(2) + angVelX(2) + angVelY(2) +
    // angVelZ(2) = 14 bytes
    processGyroIntegratedRV(&packet[4], dataLength);
  } else if ((channel == CHANNEL_REPORTS || channel == CHANNEL_WAKE_REPORTS) &&
             dataLength > 0) {
    int byteIndex = 0;
    while (byteIndex < dataLength) {
      uint8_t reportId = packet[4 + byteIndex];
      int requiredBytes = getReportLength(reportId);

      if (requiredBytes == 0 || byteIndex + requiredBytes > dataLength)
        break;

      processSensorReport(reportId, &packet[4 + byteIndex], requiredBytes);
      byteIndex += requiredBytes;
    }
  }
}

void BNO08xIMU::processGyroIntegratedRV(const uint8_t *data, int length) {
  // Gyro Integrated RV on channel 5 has NO header - raw data starts at offset 0
  // Format: i(2) + j(2) + k(2) + real(2) + angVelX(2) + angVelY(2) + angVelZ(2)
  // = 14 bytes
  if (length < 14)
    return;

  auto read16 = [](const uint8_t *p) -> int16_t {
    return (int16_t)(p[0] | (p[1] << 8));
  };

  std::lock_guard<std::mutex> lock(data_mutex_);
  gyroIntegratedRotationVector_.i = read16(&data[0]) * Q_POINT_14_SCALAR;
  gyroIntegratedRotationVector_.j = read16(&data[2]) * Q_POINT_14_SCALAR;
  gyroIntegratedRotationVector_.k = read16(&data[4]) * Q_POINT_14_SCALAR;
  gyroIntegratedRotationVector_.real = read16(&data[6]) * Q_POINT_14_SCALAR;
  gyroIntegratedRotationVector_.angVelX = read16(&data[8]) * Q_POINT_10_SCALAR;
  gyroIntegratedRotationVector_.angVelY = read16(&data[10]) * Q_POINT_10_SCALAR;
  gyroIntegratedRotationVector_.angVelZ = read16(&data[12]) * Q_POINT_10_SCALAR;
}

void BNO08xIMU::processSensorReport(uint8_t reportId, const uint8_t *data,
                                    int length) {
  // Helper to read int16 from data
  auto read16 = [](const uint8_t *p) -> int16_t {
    return (int16_t)(p[0] | (p[1] << 8));
  };

  auto readu16 = [](const uint8_t *p) -> uint16_t {
    return (uint16_t)(p[0] | (p[1] << 8));
  };

  auto readu32 = [](const uint8_t *p) -> uint32_t {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
  };

  switch (reportId) {
  case REPORT_BASE_TIMESTAMP:
    return;

  // Calibrated accelerometer-type sensors (Q8)
  case BNO_REPORT_ACCELEROMETER:
  case BNO_REPORT_LINEAR_ACCELERATION:
  case BNO_REPORT_GRAVITY: {
    if (length < 10)
      return;
    Vector3 vec(read16(&data[4]) * Q_POINT_8_SCALAR,
                read16(&data[6]) * Q_POINT_8_SCALAR,
                read16(&data[8]) * Q_POINT_8_SCALAR);

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (reportId == BNO_REPORT_ACCELEROMETER)
      accelerometer_ = vec;
    else if (reportId == BNO_REPORT_LINEAR_ACCELERATION)
      linearAcceleration_ = vec;
    else
      gravity_ = vec;
    break;
  }

  // Calibrated gyroscope (Q9)
  case BNO_REPORT_GYROSCOPE_CALIBRATED: {
    if (length < 10)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    gyroscopeCalibrated_ = Vector3(read16(&data[4]) * Q_POINT_9_SCALAR,
                                   read16(&data[6]) * Q_POINT_9_SCALAR,
                                   read16(&data[8]) * Q_POINT_9_SCALAR);
    break;
  }

  // Calibrated magnetometer (Q4)
  case BNO_REPORT_MAGNETIC_FIELD_CALIBRATED: {
    if (length < 10)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    magneticFieldCalibrated_ = Vector3(read16(&data[4]) * Q_POINT_4_SCALAR,
                                       read16(&data[6]) * Q_POINT_4_SCALAR,
                                       read16(&data[8]) * Q_POINT_4_SCALAR);
    break;
  }

  // Uncalibrated gyroscope (Q9)
  case BNO_REPORT_GYROSCOPE_UNCALIBRATED: {
    if (length < 16)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    gyroUncal_.x = read16(&data[4]) * Q_POINT_9_SCALAR;
    gyroUncal_.y = read16(&data[6]) * Q_POINT_9_SCALAR;
    gyroUncal_.z = read16(&data[8]) * Q_POINT_9_SCALAR;
    gyroUncal_.biasX = read16(&data[10]) * Q_POINT_9_SCALAR;
    gyroUncal_.biasY = read16(&data[12]) * Q_POINT_9_SCALAR;
    gyroUncal_.biasZ = read16(&data[14]) * Q_POINT_9_SCALAR;
    break;
  }

  // Uncalibrated magnetometer (Q4)
  case BNO_REPORT_MAGNETIC_FIELD_UNCALIBRATED: {
    if (length < 16)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    magUncal_.x = read16(&data[4]) * Q_POINT_4_SCALAR;
    magUncal_.y = read16(&data[6]) * Q_POINT_4_SCALAR;
    magUncal_.z = read16(&data[8]) * Q_POINT_4_SCALAR;
    magUncal_.biasX = read16(&data[10]) * Q_POINT_4_SCALAR;
    magUncal_.biasY = read16(&data[12]) * Q_POINT_4_SCALAR;
    magUncal_.biasZ = read16(&data[14]) * Q_POINT_4_SCALAR;
    break;
  }

  // Raw accelerometer
  case BNO_REPORT_RAW_ACCELEROMETER: {
    if (length < 16)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    rawAccel_.x = read16(&data[4]);
    rawAccel_.y = read16(&data[6]);
    rawAccel_.z = read16(&data[8]);
    rawAccel_.timestamp = readu32(&data[12]);
    break;
  }

  // Raw gyroscope
  case BNO_REPORT_RAW_GYROSCOPE: {
    if (length < 16)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    rawGyro_.x = read16(&data[4]);
    rawGyro_.y = read16(&data[6]);
    rawGyro_.z = read16(&data[8]);
    rawGyro_.temperature = read16(&data[10]);
    rawGyro_.timestamp = readu32(&data[12]);
    break;
  }

  // Raw magnetometer
  case BNO_REPORT_RAW_MAGNETOMETER: {
    if (length < 16)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    rawMag_.x = read16(&data[4]);
    rawMag_.y = read16(&data[6]);
    rawMag_.z = read16(&data[8]);
    rawMag_.timestamp = readu32(&data[12]);
    break;
  }

  // Rotation vector with accuracy (Q14 quaternion, Q12 accuracy)
  case BNO_REPORT_ROTATION_VECTOR:
  case BNO_REPORT_ARVR_STABILIZED_RV: {
    if (length < 14)
      return;
    Quaternion q;
    q.i = read16(&data[4]) * Q_POINT_14_SCALAR;
    q.j = read16(&data[6]) * Q_POINT_14_SCALAR;
    q.k = read16(&data[8]) * Q_POINT_14_SCALAR;
    q.real = read16(&data[10]) * Q_POINT_14_SCALAR;
    q.accuracy = read16(&data[12]) * Q_POINT_12_SCALAR;

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (reportId == BNO_REPORT_ROTATION_VECTOR)
      rotationVector_ = q;
    else
      arvrStabilizedRV_ = q;
    break;
  }

  // Geomagnetic rotation vector (Q14 quaternion, Q12 accuracy)
  case BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: {
    if (length < 14)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    geomagneticRotationVector_.i = read16(&data[4]) * Q_POINT_14_SCALAR;
    geomagneticRotationVector_.j = read16(&data[6]) * Q_POINT_14_SCALAR;
    geomagneticRotationVector_.k = read16(&data[8]) * Q_POINT_14_SCALAR;
    geomagneticRotationVector_.real = read16(&data[10]) * Q_POINT_14_SCALAR;
    geomagneticRotationVector_.accuracy = read16(&data[12]) * Q_POINT_12_SCALAR;
    break;
  }

  // Game rotation vector (Q14, no accuracy)
  case BNO_REPORT_GAME_ROTATION_VECTOR:
  case BNO_REPORT_ARVR_STABILIZED_GRV: {
    if (length < 12)
      return;
    Quaternion q;
    q.i = read16(&data[4]) * Q_POINT_14_SCALAR;
    q.j = read16(&data[6]) * Q_POINT_14_SCALAR;
    q.k = read16(&data[8]) * Q_POINT_14_SCALAR;
    q.real = read16(&data[10]) * Q_POINT_14_SCALAR;
    q.accuracy = data[2] & 0x03;

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (reportId == BNO_REPORT_GAME_ROTATION_VECTOR)
      gameRotationVector_ = q;
    else
      arvrStabilizedGRV_ = q;
    break;
  }

  // Gyro integrated rotation vector (different format - no status bytes)
  case BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: {
    if (length < 14)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    gyroIntegratedRotationVector_.i = read16(&data[0]) * Q_POINT_14_SCALAR;
    gyroIntegratedRotationVector_.j = read16(&data[2]) * Q_POINT_14_SCALAR;
    gyroIntegratedRotationVector_.k = read16(&data[4]) * Q_POINT_14_SCALAR;
    gyroIntegratedRotationVector_.real = read16(&data[6]) * Q_POINT_14_SCALAR;
    gyroIntegratedRotationVector_.angVelX =
        read16(&data[8]) * Q_POINT_10_SCALAR;
    gyroIntegratedRotationVector_.angVelY =
        read16(&data[10]) * Q_POINT_10_SCALAR;
    gyroIntegratedRotationVector_.angVelZ =
        read16(&data[12]) * Q_POINT_10_SCALAR;
    break;
  }

  // Environmental sensors
  case BNO_REPORT_PRESSURE: {
    if (length < 8)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    pressure_ = readu32(&data[4]) * Q_POINT_20_SCALAR;
    break;
  }

  case BNO_REPORT_AMBIENT_LIGHT: {
    if (length < 8)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    ambientLight_ = readu32(&data[4]) * Q_POINT_8_SCALAR;
    break;
  }

  case BNO_REPORT_HUMIDITY: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    humidity_ = read16(&data[4]) * Q_POINT_8_SCALAR;
    break;
  }

  case BNO_REPORT_PROXIMITY: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    proximity_ = read16(&data[4]) * Q_POINT_4_SCALAR;
    break;
  }

  case BNO_REPORT_TEMPERATURE: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    temperature_ = read16(&data[4]) * Q_POINT_7_SCALAR;
    break;
  }

  // Activity detectors
  case BNO_REPORT_TAP_DETECTOR: {
    if (length < 5)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    tapDetector_ = data[4];
    break;
  }

  case BNO_REPORT_STEP_DETECTOR: {
    if (length < 8)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    stepDetectorLatency_ = readu32(&data[4]);
    break;
  }

  case BNO_REPORT_STEP_COUNTER: {
    if (length < 12)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    stepCounter_.latency = readu32(&data[4]);
    stepCounter_.steps = readu16(&data[8]);
    break;
  }

  case BNO_REPORT_SIGNIFICANT_MOTION: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    significantMotion_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_STABILITY_CLASSIFIER: {
    if (length < 5)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    stabilityClassifier_ = data[4];
    break;
  }

  case BNO_REPORT_SHAKE_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    shakeDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_FLIP_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    flipDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_PICKUP_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    pickupDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_STABILITY_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    stabilityDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_PERSONAL_ACTIVITY_CLASSIFIER: {
    if (length < 16)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    activityClassifier_.mostLikelyState = data[5];
    for (int i = 0; i < 10; i++) {
      activityClassifier_.confidence[i] = data[6 + i];
    }
    break;
  }

  case BNO_REPORT_SLEEP_DETECTOR: {
    if (length < 5)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    sleepDetector_ = data[4];
    break;
  }

  case BNO_REPORT_TILT_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    tiltDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_POCKET_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    pocketDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_CIRCLE_DETECTOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    circleDetector_ = readu16(&data[4]);
    break;
  }

  case BNO_REPORT_HEART_RATE_MONITOR: {
    if (length < 6)
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    heartRate_ = readu16(&data[4]);
    break;
  }

  default:
    break;
  }
}

void BNO08xIMU::readThreadFunc() {
  uint8_t buffer[MAX_PACKET_SIZE];
  int length;

  while (running_) {
    if (readPacket(buffer, length)) {
      processPacket(buffer, length);
    }
    usleep(1000);
  }
}

bool BNO08xIMU::waitForData(int timeoutMs) {
  fd_set readfds;
  struct timeval timeout;

  timeout.tv_sec = timeoutMs / 1000;
  timeout.tv_usec = (timeoutMs % 1000) * 1000;

  FD_ZERO(&readfds);
  FD_SET(uart_fd_, &readfds);

  return select(uart_fd_ + 1, &readfds, NULL, NULL, &timeout) > 0;
}

bool BNO08xIMU::softReset() {
  uint8_t buffer[MAX_PACKET_SIZE];
  int length;

  int drained = 0;
  while (waitForData(100) && drained < 100) {
    if (readPacket(buffer, length))
      drained++;
  }

  tcflush(uart_fd_, TCIOFLUSH);
  usleep(100000);

  uint8_t exe_cmd[1] = {1};
  sendPacket(CHANNEL_EXE, exe_cmd, 1);
  usleep(300000);
  sendPacket(CHANNEL_EXE, exe_cmd, 1);
  usleep(300000);
  sendPacket(CHANNEL_EXE, exe_cmd, 1);
  usleep(500000);

  drained = 0;
  while (waitForData(100) && drained < 50) {
    if (readPacket(buffer, length))
      drained++;
  }

  tcflush(uart_fd_, TCIOFLUSH);
  usleep(200000);

  uint8_t shtp_cmd[2] = {0, 1};
  sendPacket(CHANNEL_SHTP_COMMAND, shtp_cmd, 2);
  usleep(500000);

  bool got_shtp_response = false;
  for (int i = 0; i < 15; i++) {
    if (waitForData(500)) {
      if (readPacket(buffer, length)) {
        if (buffer[2] == CHANNEL_SHTP_COMMAND) {
          got_shtp_response = true;
          break;
        }
      }
    } else {
      sendPacket(CHANNEL_SHTP_COMMAND, shtp_cmd, 2);
      usleep(500000);
    }
  }

  if (!got_shtp_response)
    return false;

  sendPacket(CHANNEL_EXE, exe_cmd, 1);
  usleep(500000);
  sendPacket(CHANNEL_EXE, exe_cmd, 1);
  usleep(500000);

  drained = 0;
  while (waitForData(100) && drained < 20) {
    if (readPacket(buffer, length))
      drained++;
  }

  return true;
}

bool BNO08xIMU::checkProductId() {
  uint8_t request[2] = {REPORT_PRODUCT_ID_REQUEST, 0};
  sendPacket(CHANNEL_CONTROL, request, 2);

  uint8_t buffer[MAX_PACKET_SIZE];
  int length;

  for (int i = 0; i < 10; i++) {
    if (waitForData(500) && readPacket(buffer, length)) {
      if (length > SHTP_HEADER_SIZE && buffer[4] == REPORT_PRODUCT_ID_RESPONSE)
        return true;
    }
  }

  return false;
}

void BNO08xIMU::setFeature(uint8_t reportId, uint32_t interval_us) {
  uint8_t command[17];
  memset(command, 0, sizeof(command));

  command[0] = REPORT_SET_FEATURE;
  command[1] = reportId;
  command[5] = (interval_us >> 0) & 0xFF;
  command[6] = (interval_us >> 8) & 0xFF;
  command[7] = (interval_us >> 16) & 0xFF;
  command[8] = (interval_us >> 24) & 0xFF;

  sendPacket(CHANNEL_CONTROL, command, sizeof(command));

  if (interval_us == 0) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    enabled_features_.erase(reportId);
    return;
  }

  auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(2);

  while (true) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout)
      break;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (enabled_features_.find(reportId) != enabled_features_.end())
        break;
    }

    usleep(10000);
  }
}

void BNO08xIMU::enableFeature(BNO_REPORT feature, uint32_t interval_us) {
  setFeature(static_cast<uint8_t>(feature), interval_us);
}

void BNO08xIMU::disableFeature(BNO_REPORT feature) {
  setFeature(static_cast<uint8_t>(feature), 0);
}

// Calibrated sensor getters
Vector3 BNO08xIMU::accelerometer() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return accelerometer_;
}

Vector3 BNO08xIMU::gyroscopeCalibrated() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return gyroscopeCalibrated_;
}

Vector3 BNO08xIMU::magneticFieldCalibrated() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return magneticFieldCalibrated_;
}

Vector3 BNO08xIMU::linearAcceleration() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return linearAcceleration_;
}

Vector3 BNO08xIMU::gravity() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return gravity_;
}

// Rotation vector getters
Quaternion BNO08xIMU::rotationVector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return rotationVector_;
}

Quaternion BNO08xIMU::gameRotationVector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return gameRotationVector_;
}

Quaternion BNO08xIMU::geomagneticRotationVector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return geomagneticRotationVector_;
}

Quaternion BNO08xIMU::arvrStabilizedRV() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return arvrStabilizedRV_;
}

Quaternion BNO08xIMU::arvrStabilizedGRV() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return arvrStabilizedGRV_;
}

GyroIntegratedRV BNO08xIMU::gyroIntegratedRotationVector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return gyroIntegratedRotationVector_;
}

// Uncalibrated sensor getters
Vector3WithBias BNO08xIMU::gyroUncalibrated() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return gyroUncal_;
}

Vector3WithBias BNO08xIMU::magneticFieldUncalibrated() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return magUncal_;
}

// Raw sensor getters
RawVector3 BNO08xIMU::rawAccelerometer() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return rawAccel_;
}

RawGyroscope BNO08xIMU::rawGyroscope() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return rawGyro_;
}

RawVector3 BNO08xIMU::rawMagnetometer() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return rawMag_;
}

// Environmental getters
float BNO08xIMU::pressure() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return pressure_;
}

float BNO08xIMU::ambientLight() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ambientLight_;
}

float BNO08xIMU::humidity() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return humidity_;
}

float BNO08xIMU::proximity() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return proximity_;
}

float BNO08xIMU::temperature() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return temperature_;
}

// Activity detector getters
uint8_t BNO08xIMU::tapDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return tapDetector_;
}

StepCounter BNO08xIMU::stepCounter() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return stepCounter_;
}

uint32_t BNO08xIMU::stepDetectorLatency() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return stepDetectorLatency_;
}

uint16_t BNO08xIMU::significantMotion() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return significantMotion_;
}

uint8_t BNO08xIMU::stabilityClassifier() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return stabilityClassifier_;
}

uint16_t BNO08xIMU::shakeDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return shakeDetector_;
}

uint16_t BNO08xIMU::flipDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return flipDetector_;
}

uint16_t BNO08xIMU::pickupDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return pickupDetector_;
}

uint16_t BNO08xIMU::stabilityDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return stabilityDetector_;
}

ActivityClassifier BNO08xIMU::activityClassifier() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return activityClassifier_;
}

uint8_t BNO08xIMU::sleepDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return sleepDetector_;
}

uint16_t BNO08xIMU::tiltDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return tiltDetector_;
}

uint16_t BNO08xIMU::pocketDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return pocketDetector_;
}

uint16_t BNO08xIMU::circleDetector() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return circleDetector_;
}

uint16_t BNO08xIMU::heartRate() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return heartRate_;
}

} // namespace imu_sdk
