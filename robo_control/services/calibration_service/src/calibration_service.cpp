#include "calibration_service/calibration_service.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include <spdlog/spdlog.h>
#include <transport/node_context.hpp>
#include <yaml-cpp/yaml.h>

namespace calibration_service {

CalibrationService::CalibrationService() {}

CalibrationService::~CalibrationService() { stop(); }

const char *CalibrationService::referenceLabel(CalibrationReference r) {
  switch (r) {
    case CalibrationReference::Lower: return "lower";
    case CalibrationReference::Upper: return "upper";
    default: return "zero";
  }
}

void CalibrationService::loadParameters() {
  auto node = transport::NodeContext::instance().node();
  config_path_ = node->declare_parameter<std::string>("config_path");
}

void CalibrationService::loadConfig() {
  hardware_config_path_ = config_path_;
  auto dot = config_path_.find_last_of('.');
  output_path_ = (dot == std::string::npos ? config_path_
                                           : config_path_.substr(0, dot)) +
                 ".staging.yaml";

  YAML::Node root = YAML::LoadFile(hardware_config_path_);

  std::string hardware_type =
      root["hardware_type"] ? root["hardware_type"].as<std::string>() : "";
  if (hardware_type != "ethercat") {
    throw std::runtime_error(
        "calibration_service only supports hardware_type=ethercat (got '" +
        hardware_type + "')");
  }
  robot_type_ =
      root["robot_type"] ? root["robot_type"].as<std::string>() : "";

  YAML::Node actuators = root["actuators"];
  if (!actuators || !actuators.IsSequence() || actuators.size() == 0) {
    throw std::runtime_error(
        "Missing or empty top-level 'actuators' list in hardware config");
  }

  entries_.clear();
  entries_.reserve(actuators.size());
  std::unordered_map<int, std::size_t> id_to_index;
  for (const auto &a : actuators) {
    ActuatorEntry e;
    e.id = a["id"].as<int>();
    if (!a["name"]) {
      throw std::runtime_error("Actuator id " + std::to_string(e.id) +
                               " missing 'name'");
    }
    e.name = a["name"].as<std::string>();
    e.type = a["type"].as<std::string>();
    e.direction = a["direction"].as<int>();
    e.old_offset = a["zero_offset"] ? a["zero_offset"].as<double>() : 0.0;
    e.lower_limit = a["lower_limit"] ? a["lower_limit"].as<double>() : 0.0;
    e.upper_limit = a["upper_limit"] ? a["upper_limit"].as<double>() : 0.0;

    std::string ref =
        a["calibration_reference"]
            ? a["calibration_reference"].as<std::string>()
            : "zero";
    if (ref == "zero") {
      e.reference = CalibrationReference::Zero;
    } else if (ref == "lower") {
      e.reference = CalibrationReference::Lower;
    } else if (ref == "upper") {
      e.reference = CalibrationReference::Upper;
    } else {
      throw std::runtime_error(
          "Actuator '" + e.name +
          "': calibration_reference must be 'zero', 'lower', or 'upper' (got '" +
          ref + "')");
    }

    if (!id_to_index.emplace(e.id, entries_.size()).second) {
      throw std::runtime_error("Duplicate actuator id " + std::to_string(e.id));
    }
    entries_.push_back(std::move(e));
  }

  YAML::Node ec = root["ethercat"];
  if (!ec) {
    throw std::runtime_error("Missing 'ethercat' section in hardware config");
  }
  cycle_time_us_ =
      ec["cycle_time_us"] ? ec["cycle_time_us"].as<int>() : 1000;

  YAML::Node interfaces = ec["interfaces"];
  if (!interfaces || !interfaces.IsSequence() || interfaces.size() == 0) {
    throw std::runtime_error("'ethercat.interfaces' is missing or empty");
  }
  if (interfaces.size() > 1) {
    spdlog::warn(
        "Multiple EtherCAT interfaces declared; calibration uses the first ('{}')",
        interfaces[0]["name"].as<std::string>());
  }
  const auto &iface = interfaces[0];
  ethercat_interface_ = iface["name"].as<std::string>();

  id_to_bus_id_.clear();
  actuator_type_by_bus_id_.clear();
  for (const auto &entry : iface["actuators"]) {
    int id = entry["id"].as<int>();
    int bus_id = entry["bus_id"].as<int>();
    auto it = id_to_index.find(id);
    if (it == id_to_index.end()) {
      throw std::runtime_error("Interface references unknown actuator id " +
                               std::to_string(id));
    }
    id_to_bus_id_[id] = bus_id;
    actuator_type_by_bus_id_[bus_id] = entries_[it->second].type;
  }

  spdlog::info("Calibration entries: {} actuator(s) on interface '{}'",
               entries_.size(), ethercat_interface_);
  for (const auto &e : entries_) {
    spdlog::info(
        "  id={:>2} name={:<18} type={:<8} dir={:+d} old_offset={:+.4f} "
        "limits=[{:+.4f}, {:+.4f}] ref={}",
        e.id, e.name, e.type, e.direction, e.old_offset, e.lower_limit,
        e.upper_limit, referenceLabel(e.reference));
  }
}

bool CalibrationService::init() {
  loadParameters();

  if (config_path_.empty()) {
    spdlog::error("No config_path provided");
    return false;
  }

  try {
    loadConfig();
  } catch (const std::exception &e) {
    spdlog::error("Failed to load calibration config: {}", e.what());
    return false;
  }

  auto node = transport::NodeContext::instance().node();
  calibrate_service_ =
      node->create_service<robo_msgs::srv::CalibrateActuators>(
          "calibrate_actuators",
          std::bind(&CalibrationService::handleCalibrate, this,
                    std::placeholders::_1, std::placeholders::_2));
  get_service_ = node->create_service<robo_msgs::srv::GetActuatorOffsets>(
      "get_actuator_offsets",
      std::bind(&CalibrationService::handleGet, this, std::placeholders::_1,
                std::placeholders::_2));

  spdlog::info("Calibration Service initialized ({} actuator(s))",
               entries_.size());
  return true;
}

void CalibrationService::start() {
  if (!startEtherCAT()) {
    spdlog::error("Failed to start EtherCAT; calibration cannot serve requests");
    return;
  }
  spdlog::info("Calibration Service ready");
}

void CalibrationService::stop() {
  stopEtherCAT();
  calibrate_service_.reset();
  get_service_.reset();
}

bool CalibrationService::startEtherCAT() {
  master_ = std::make_unique<ethercat_sdk::EtherCATMaster>(ethercat_interface_,
                                                            cycle_time_us_);
  master_->configActuatorTypes(actuator_type_by_bus_id_);
  if (!master_->init()) {
    spdlog::error("EtherCAT init failed on interface '{}'", ethercat_interface_);
    master_.reset();
    return false;
  }
  int slave_count = master_->getSlaveCount();
  if (slave_count != static_cast<int>(actuator_type_by_bus_id_.size())) {
    spdlog::error("EtherCAT slave count mismatch: found {}, expected {}",
                  slave_count, actuator_type_by_bus_id_.size());
    master_.reset();
    return false;
  }
  master_->setOperationMode(ethercat_sdk::OperationMode::PVT);
  master_->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  spdlog::info("EtherCAT up on '{}': {} slave(s), PVT zero-gain mode",
               ethercat_interface_, slave_count);
  return true;
}

void CalibrationService::stopEtherCAT() {
  if (master_) {
    master_->stop();
    master_.reset();
  }
}

bool CalibrationService::resolveSelection(
    const std::vector<std::string> &names,
    std::vector<std::size_t> &selected, std::string &error_out) const {
  selected.clear();
  if (names.empty()) {
    selected.reserve(entries_.size());
    for (std::size_t i = 0; i < entries_.size(); ++i) selected.push_back(i);
    return true;
  }
  std::unordered_map<std::string, std::size_t> name_to_index;
  name_to_index.reserve(entries_.size());
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    name_to_index.emplace(entries_[i].name, i);
  }
  std::set<std::size_t> picked;
  std::vector<std::string> unknown;
  for (const auto &name : names) {
    auto it = name_to_index.find(name);
    if (it == name_to_index.end()) {
      unknown.push_back(name);
    } else {
      picked.insert(it->second);
    }
  }
  if (!unknown.empty()) {
    std::ostringstream os;
    os << "Unknown actuator name(s):";
    for (const auto &n : unknown) os << " " << n;
    error_out = os.str();
    return false;
  }
  selected.assign(picked.begin(), picked.end());
  return true;
}

void CalibrationService::handleCalibrate(
    const std::shared_ptr<robo_msgs::srv::CalibrateActuators::Request> request,
    std::shared_ptr<robo_msgs::srv::CalibrateActuators::Response> response) {

  const std::string &command = request->command;
  spdlog::info("Received command: {}", command);

  if (command == "calibrate") {
    calibrate(request, response);
  } else if (command == "apply") {
    apply(response);
  } else {
    response->success = false;
    response->message =
        "Unknown command: " + command + ". Valid commands: calibrate, apply";
  }
}

void CalibrationService::calibrate(
    const std::shared_ptr<robo_msgs::srv::CalibrateActuators::Request> request,
    std::shared_ptr<robo_msgs::srv::CalibrateActuators::Response> response) {

  if (!master_) {
    response->success = false;
    response->message = "EtherCAT not running";
    return;
  }

  std::vector<std::size_t> selected;
  std::string err;
  if (!resolveSelection(request->actuator_names, selected, err)) {
    response->success = false;
    response->message = err;
    return;
  }

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(4);

  response->offsets.reserve(selected.size());
  response->actuator_names.reserve(selected.size());

  for (std::size_t i : selected) {
    auto &e = entries_[i];
    auto bus_it = id_to_bus_id_.find(e.id);
    if (bus_it == id_to_bus_id_.end()) {
      response->success = false;
      response->message = "Actuator '" + e.name + "' has no bus_id mapping";
      return;
    }
    auto state = master_->getActuatorState(bus_it->second);
    double signed_pos = state.position * static_cast<double>(e.direction);
    double q_ref = 0.0;
    switch (e.reference) {
      case CalibrationReference::Zero: q_ref = 0.0; break;
      case CalibrationReference::Lower: q_ref = e.lower_limit; break;
      case CalibrationReference::Upper: q_ref = e.upper_limit; break;
    }
    e.new_offset = signed_pos - q_ref;

    if (signed_pos < e.lower_limit || signed_pos > e.upper_limit) {
      spdlog::warn(
          "{} measured {:.4f} outside declared motor-space limits [{:.4f}, {:.4f}]",
          e.name, signed_pos, e.lower_limit, e.upper_limit);
    }

    summary << "\n  " << e.name << " (id=" << e.id
            << "): measured=" << signed_pos
            << " ref=" << referenceLabel(e.reference) << "(" << q_ref
            << ") -> new_offset=" << e.new_offset << " (was " << e.old_offset
            << ")";

    response->offsets.push_back(e.new_offset);
    response->actuator_names.push_back(e.name);
  }

  try {
    writeOffsetsToStaging(selected);
  } catch (const std::exception &ex) {
    response->success = false;
    response->message =
        std::string("Calibration computed but failed to write staging file: ") +
        ex.what();
    spdlog::error("{}", response->message);
    return;
  }

  response->success = true;
  response->message = "Calibrated " + std::to_string(selected.size()) + " of " +
                      std::to_string(entries_.size()) +
                      " actuator(s). Offsets written to " + output_path_ +
                      ". Use 'apply' command to write to hardware.yaml." +
                      summary.str();

  spdlog::info("Calibration complete: {}", response->message);
}

void CalibrationService::writeOffsetsToStaging(
    const std::vector<std::size_t> &selected) {
  std::unordered_map<std::string, double> prior_offsets;
  {
    std::ifstream fin(output_path_);
    if (fin.good()) {
      try {
        YAML::Node existing = YAML::LoadFile(output_path_);
        if (existing["offsets"] && existing["offsets"].IsSequence()) {
          for (const auto &n : existing["offsets"]) {
            if (n["actuator_name"] && n["new_offset"]) {
              prior_offsets[n["actuator_name"].as<std::string>()] =
                  n["new_offset"].as<double>();
            }
          }
        }
      } catch (const std::exception &ex) {
        spdlog::warn("Ignoring unreadable existing staging file {}: {}",
                     output_path_, ex.what());
      }
    }
  }

  std::set<std::size_t> selected_set(selected.begin(), selected.end());

  YAML::Emitter out;
  out << YAML::Comment("Auto-generated by calibration_service");

  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  gmtime_r(&t, &tm_buf);
  std::ostringstream ts;
  ts << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%SZ");

  out << YAML::BeginMap;
  out << YAML::Key << "timestamp" << YAML::Value << ts.str();
  out << YAML::Key << "hardware_config_path" << YAML::Value
      << hardware_config_path_;

  out << YAML::Key << "offsets" << YAML::Value << YAML::BeginSeq;
  std::size_t written = 0;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const auto &e = entries_[i];
    bool from_now = selected_set.count(i) > 0;
    auto pit = prior_offsets.find(e.name);
    bool from_prior = !from_now && pit != prior_offsets.end();
    if (!from_now && !from_prior) continue;

    double new_offset = from_now ? e.new_offset : pit->second;

    out << YAML::BeginMap;
    out << YAML::Key << "actuator_id" << YAML::Value << e.id;
    out << YAML::Key << "actuator_name" << YAML::Value << e.name;
    out << YAML::Key << "old_offset" << YAML::Value << e.old_offset;
    out << YAML::Key << "new_offset" << YAML::Value << new_offset;
    out << YAML::Key << "lower_limit" << YAML::Value << e.lower_limit;
    out << YAML::Key << "upper_limit" << YAML::Value << e.upper_limit;
    out << YAML::Key << "calibration_reference" << YAML::Value
        << referenceLabel(e.reference);
    out << YAML::EndMap;
    ++written;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(output_path_);
  if (!fout.is_open()) {
    throw std::runtime_error("Failed to open staging file: " + output_path_);
  }
  fout << out.c_str() << "\n";

  spdlog::info("Wrote {} calibration offset(s) to {}", written, output_path_);
}

void CalibrationService::apply(
    std::shared_ptr<robo_msgs::srv::CalibrateActuators::Response> response) {
  try {
    applyOffsetsToHardwareYaml();
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    spdlog::error("{}", response->message);
    return;
  }

  response->success = true;
  response->message = "Offsets applied to " + hardware_config_path_ +
                      ". Staging cleared." +
                      " Restart control_service to use new offsets.";
  spdlog::info("{}", response->message);
}

void CalibrationService::applyOffsetsToHardwareYaml() {
  YAML::Node staging;
  try {
    staging = YAML::LoadFile(output_path_);
  } catch (const YAML::BadFile &) {
    throw std::runtime_error("Staging file not found: " + output_path_ +
                             ". Run 'calibrate' first.");
  }

  if (!staging["offsets"]) {
    throw std::runtime_error(
        "No offsets found in staging file. Run 'calibrate' first.");
  }

  std::unordered_map<std::string, double> offset_by_name;
  for (const auto &entry : staging["offsets"]) {
    offset_by_name[entry["actuator_name"].as<std::string>()] =
        entry["new_offset"].as<double>();
  }

  YAML::Node hw_root = YAML::LoadFile(hardware_config_path_);
  YAML::Node actuators = hw_root["actuators"];
  if (!actuators || !actuators.IsSequence() || actuators.size() == 0) {
    throw std::runtime_error(
        "Missing or empty top-level 'actuators' list in hardware config");
  }

  int updated = 0;
  for (auto actuator : actuators) {
    if (!actuator["name"]) continue;
    std::string name = actuator["name"].as<std::string>();
    auto it = offset_by_name.find(name);
    if (it != offset_by_name.end()) {
      actuator["zero_offset"] = it->second;
      updated++;
      spdlog::info("Updated {}: zero_offset = {:.6f}", name, it->second);
    }
  }

  if (updated == 0) {
    throw std::runtime_error("No actuators were updated in hardware.yaml");
  }

  std::ofstream fout(hardware_config_path_);
  if (!fout.is_open()) {
    throw std::runtime_error("Failed to open hardware config for writing: " +
                             hardware_config_path_);
  }
  fout << hw_root;
  fout.close();

  for (auto &e : entries_) {
    auto it = offset_by_name.find(e.name);
    if (it != offset_by_name.end()) e.old_offset = it->second;
  }

  if (std::remove(output_path_.c_str()) != 0) {
    spdlog::warn("Applied offsets, but failed to remove staging file {}",
                 output_path_);
  }

  spdlog::info("Applied {} offset(s) to {}", updated, hardware_config_path_);
}

void CalibrationService::handleGet(
    const std::shared_ptr<robo_msgs::srv::GetActuatorOffsets::Request> request,
    std::shared_ptr<robo_msgs::srv::GetActuatorOffsets::Response> response) {
  std::vector<std::size_t> selected;
  std::string err;
  if (!resolveSelection(request->actuator_names, selected, err)) {
    response->success = false;
    response->message = err;
    return;
  }

  response->actuator_names.reserve(selected.size());
  response->offsets.reserve(selected.size());
  response->calibration_references.reserve(selected.size());
  for (std::size_t i : selected) {
    const auto &e = entries_[i];
    response->actuator_names.push_back(e.name);
    response->offsets.push_back(e.old_offset);
    response->calibration_references.push_back(referenceLabel(e.reference));
  }
  response->success = true;
  response->message = "Returned " + std::to_string(selected.size()) +
                      " actuator offset(s)";
}

} // namespace calibration_service
