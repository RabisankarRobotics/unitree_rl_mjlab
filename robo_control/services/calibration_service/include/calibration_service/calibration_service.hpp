#pragma once

#include "robo_msgs/srv/calibrate_actuators.hpp"
#include "robo_msgs/srv/get_actuator_offsets.hpp"

#include <ethercat_sdk/master.hpp>

#include <cstddef>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace calibration_service {

enum class CalibrationReference { Zero, Lower, Upper };

struct ActuatorEntry {
  int id;
  std::string name;
  std::string type;
  int direction = 1;
  double old_offset = 0.0;
  double new_offset = 0.0;
  double lower_limit = 0.0;
  double upper_limit = 0.0;
  CalibrationReference reference = CalibrationReference::Zero;
};

class CalibrationService {
public:
  CalibrationService();
  ~CalibrationService();

  CalibrationService(const CalibrationService &) = delete;
  CalibrationService &operator=(const CalibrationService &) = delete;

  bool init();
  void start();
  void stop();

private:
  void loadParameters();
  void loadConfig();

  void handleCalibrate(
      const std::shared_ptr<robo_msgs::srv::CalibrateActuators::Request> request,
      std::shared_ptr<robo_msgs::srv::CalibrateActuators::Response> response);

  void handleGet(
      const std::shared_ptr<robo_msgs::srv::GetActuatorOffsets::Request> request,
      std::shared_ptr<robo_msgs::srv::GetActuatorOffsets::Response> response);

  void calibrate(
      const std::shared_ptr<robo_msgs::srv::CalibrateActuators::Request> request,
      std::shared_ptr<robo_msgs::srv::CalibrateActuators::Response> response);
  void apply(
      std::shared_ptr<robo_msgs::srv::CalibrateActuators::Response> response);

  bool startEtherCAT();
  void stopEtherCAT();

  void writeOffsetsToStaging(const std::vector<std::size_t> &selected);
  void applyOffsetsToHardwareYaml();

  bool resolveSelection(const std::vector<std::string> &names,
                        std::vector<std::size_t> &selected,
                        std::string &error_out) const;

  static const char *referenceLabel(CalibrationReference r);

  std::string config_path_;
  std::string hardware_config_path_;
  std::string output_path_;

  std::string robot_type_;
  std::string ethercat_interface_;
  int cycle_time_us_ = 1000;

  std::vector<ActuatorEntry> entries_;
  std::unordered_map<int, int> id_to_bus_id_;
  std::map<int, std::string> actuator_type_by_bus_id_;

  std::unique_ptr<ethercat_sdk::EtherCATMaster> master_;

  rclcpp::Service<robo_msgs::srv::CalibrateActuators>::SharedPtr calibrate_service_;
  rclcpp::Service<robo_msgs::srv::GetActuatorOffsets>::SharedPtr get_service_;
};

} // namespace calibration_service
