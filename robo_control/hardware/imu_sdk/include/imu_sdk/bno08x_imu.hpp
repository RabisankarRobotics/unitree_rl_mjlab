#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <thread>

namespace imu_sdk {

// Sensor feature report IDs
enum BNO_REPORT {
  // Core calibrated sensors
  BNO_REPORT_ACCELEROMETER = 0x01,
  BNO_REPORT_GYROSCOPE_CALIBRATED = 0x02,
  BNO_REPORT_MAGNETIC_FIELD_CALIBRATED = 0x03,
  BNO_REPORT_LINEAR_ACCELERATION = 0x04,
  BNO_REPORT_ROTATION_VECTOR = 0x05,
  BNO_REPORT_GRAVITY = 0x06,
  BNO_REPORT_GYROSCOPE_UNCALIBRATED = 0x07,
  BNO_REPORT_GAME_ROTATION_VECTOR = 0x08,
  BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09,

  // Environmental sensors
  BNO_REPORT_PRESSURE = 0x0A,
  BNO_REPORT_AMBIENT_LIGHT = 0x0B,
  BNO_REPORT_HUMIDITY = 0x0C,
  BNO_REPORT_PROXIMITY = 0x0D,
  BNO_REPORT_TEMPERATURE = 0x0E,
  BNO_REPORT_MAGNETIC_FIELD_UNCALIBRATED = 0x0F,

  // Activity/Event detectors
  BNO_REPORT_TAP_DETECTOR = 0x10,
  BNO_REPORT_STEP_COUNTER = 0x11,
  BNO_REPORT_SIGNIFICANT_MOTION = 0x12,
  BNO_REPORT_STABILITY_CLASSIFIER = 0x13,

  // Raw sensors
  BNO_REPORT_RAW_ACCELEROMETER = 0x14,
  BNO_REPORT_RAW_GYROSCOPE = 0x15,
  BNO_REPORT_RAW_MAGNETOMETER = 0x16,

  // More detectors
  BNO_REPORT_STEP_DETECTOR = 0x18,
  BNO_REPORT_SHAKE_DETECTOR = 0x19,
  BNO_REPORT_FLIP_DETECTOR = 0x1A,
  BNO_REPORT_PICKUP_DETECTOR = 0x1B,
  BNO_REPORT_STABILITY_DETECTOR = 0x1C,
  BNO_REPORT_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E,
  BNO_REPORT_SLEEP_DETECTOR = 0x1F,
  BNO_REPORT_TILT_DETECTOR = 0x20,
  BNO_REPORT_POCKET_DETECTOR = 0x21,
  BNO_REPORT_CIRCLE_DETECTOR = 0x22,
  BNO_REPORT_HEART_RATE_MONITOR = 0x23,

  // AR/VR reports
  BNO_REPORT_ARVR_STABILIZED_RV = 0x28,
  BNO_REPORT_ARVR_STABILIZED_GRV = 0x29,
  BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A,
  BNO_REPORT_IZRO_MOTION_REQUEST = 0x2B
};

// Stability classifier values
enum StabilityClassification {
  STABILITY_UNKNOWN = 0,
  STABILITY_ON_TABLE = 1,
  STABILITY_STATIONARY = 2,
  STABILITY_STABLE = 3,
  STABILITY_MOTION = 4
};

// Tap detector flags
enum TapFlags {
  TAP_X = 1,
  TAP_X_POS = 2,
  TAP_Y = 4,
  TAP_Y_POS = 8,
  TAP_Z = 16,
  TAP_Z_POS = 32,
  TAP_DOUBLE = 64
};

// Shake detector flags
enum ShakeFlags { SHAKE_X = 1, SHAKE_Y = 2, SHAKE_Z = 4 };

// Data structures
struct Vector3 {
  float x, y, z;

  Vector3() : x(0), y(0), z(0) {}
  Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct RawVector3 {
  int16_t x, y, z;
  uint32_t timestamp;

  RawVector3() : x(0), y(0), z(0), timestamp(0) {}
};

struct RawGyroscope {
  int16_t x, y, z;
  int16_t temperature;
  uint32_t timestamp;

  RawGyroscope() : x(0), y(0), z(0), temperature(0), timestamp(0) {}
};

struct Vector3WithBias {
  float x, y, z;
  float biasX, biasY, biasZ;

  Vector3WithBias() : x(0), y(0), z(0), biasX(0), biasY(0), biasZ(0) {}
};

struct Quaternion {
  float i, j, k, real;
  float accuracy;

  Quaternion() : i(0), j(0), k(0), real(1), accuracy(0) {}
};

struct GyroIntegratedRV {
  float i, j, k, real;
  float angVelX, angVelY, angVelZ;

  GyroIntegratedRV()
      : i(0), j(0), k(0), real(1), angVelX(0), angVelY(0), angVelZ(0) {}
};

struct StepCounter {
  uint32_t latency;
  uint16_t steps;

  StepCounter() : latency(0), steps(0) {}
};

struct ActivityClassifier {
  uint8_t mostLikelyState;
  uint8_t confidence[10];

  ActivityClassifier() : mostLikelyState(0) {
    for (int i = 0; i < 10; i++)
      confidence[i] = 0;
  }
};

class BNO08xIMU {
private:
  int uart_fd_;
  uint8_t sequence_number_[6];
  std::thread read_thread_;
  std::atomic<bool> running_;
  mutable std::mutex data_mutex_;

  // Latest sensor readings - Calibrated
  Vector3 accelerometer_;
  Vector3 gyroscopeCalibrated_;
  Vector3 magneticFieldCalibrated_;
  Vector3 linearAcceleration_;
  Vector3 gravity_;

  // Rotation vectors
  Quaternion rotationVector_;
  Quaternion gameRotationVector_;
  Quaternion geomagneticRotationVector_;
  Quaternion arvrStabilizedRV_;
  Quaternion arvrStabilizedGRV_;
  GyroIntegratedRV gyroIntegratedRotationVector_;

  // Uncalibrated sensors
  Vector3WithBias gyroUncal_;
  Vector3WithBias magUncal_;

  // Raw sensors
  RawVector3 rawAccel_;
  RawGyroscope rawGyro_;
  RawVector3 rawMag_;

  // Environmental
  float pressure_;
  float ambientLight_;
  float humidity_;
  float proximity_;
  float temperature_;

  // Activity/Event detectors
  uint8_t tapDetector_;
  StepCounter stepCounter_;
  uint32_t stepDetectorLatency_;
  uint16_t significantMotion_;
  uint8_t stabilityClassifier_;
  uint16_t shakeDetector_;
  uint16_t flipDetector_;
  uint16_t pickupDetector_;
  uint16_t stabilityDetector_;
  ActivityClassifier activityClassifier_;
  uint8_t sleepDetector_;
  uint16_t tiltDetector_;
  uint16_t pocketDetector_;
  uint16_t circleDetector_;
  uint16_t heartRate_;

  // Track enabled features
  std::set<uint8_t> enabled_features_;

  // Internal methods
  bool sendPacket(uint8_t channel, const uint8_t *data, uint16_t length);
  bool readPacket(uint8_t *buffer, int &length);
  void setFeature(uint8_t reportId, uint32_t interval_us);
  void processPacket(const uint8_t *packet, int length);
  void processGyroIntegratedRV(const uint8_t *data, int length);
  void processSensorReport(uint8_t reportId, const uint8_t *data, int length);
  void readThreadFunc();
  bool waitForData(int timeoutMs);
  bool softReset();
  bool checkProductId();

public:
  BNO08xIMU(const std::string &device, int baudrate = 3000000);
  ~BNO08xIMU();

  void enableFeature(BNO_REPORT feature, uint32_t interval_us = 10000);
  void disableFeature(BNO_REPORT feature);

  // Calibrated sensors
  Vector3 accelerometer() const;
  Vector3 gyroscopeCalibrated() const;
  Vector3 magneticFieldCalibrated() const;
  Vector3 linearAcceleration() const;
  Vector3 gravity() const;

  // Rotation vectors
  Quaternion rotationVector() const;
  Quaternion gameRotationVector() const;
  Quaternion geomagneticRotationVector() const;
  Quaternion arvrStabilizedRV() const;
  Quaternion arvrStabilizedGRV() const;
  GyroIntegratedRV gyroIntegratedRotationVector() const;

  // Uncalibrated sensors
  Vector3WithBias gyroUncalibrated() const;
  Vector3WithBias magneticFieldUncalibrated() const;

  // Raw sensors
  RawVector3 rawAccelerometer() const;
  RawGyroscope rawGyroscope() const;
  RawVector3 rawMagnetometer() const;

  // Environmental
  float pressure() const;
  float ambientLight() const;
  float humidity() const;
  float proximity() const;
  float temperature() const;

  // Activity/Event detectors
  uint8_t tapDetector() const;
  StepCounter stepCounter() const;
  uint32_t stepDetectorLatency() const;
  uint16_t significantMotion() const;
  uint8_t stabilityClassifier() const;
  uint16_t shakeDetector() const;
  uint16_t flipDetector() const;
  uint16_t pickupDetector() const;
  uint16_t stabilityDetector() const;
  ActivityClassifier activityClassifier() const;
  uint8_t sleepDetector() const;
  uint16_t tiltDetector() const;
  uint16_t pocketDetector() const;
  uint16_t circleDetector() const;
  uint16_t heartRate() const;
};

} // namespace imu_sdk
