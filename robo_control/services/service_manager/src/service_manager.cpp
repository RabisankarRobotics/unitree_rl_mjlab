#include "rclcpp/rclcpp.hpp"
#include "robo_msgs/srv/manage_service.hpp"
#include <cstdlib>
#include <map>
#include <string>
#include <thread>

#include "common/config_loader.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

struct ServiceInfo {
  std::string package;
  std::string executable;
  std::string params;
};

class ServiceManager : public rclcpp::Node {
public:
  ServiceManager() : Node("service_manager") {
    this->declare_parameter("config_path", "");
    std::string config_path = this->get_parameter("config_path").as_string();

    if (config_path.empty()) {
      spdlog::error("No config_path provided");
      return;
    }

    YAML::Node config = YAML::LoadFile(config_path);

    std::vector<std::string> service_names =
        common::get_config_value<std::vector<std::string>>(config,
                                                           "service_names");

    for (const auto &name : service_names) {
      if (!config[name]) {
        spdlog::error("Missing config for service: {}", name);
        continue;
      }
      YAML::Node service_node = config[name];

      std::string package =
          common::get_config_value<std::string>(service_node, "package");
      std::string executable =
          common::get_config_value<std::string>(service_node, "executable");
      std::string params =
          common::get_config_value<std::string>(service_node, "command_params");

      services_[name] = {package, executable, params};
      spdlog::info("Loaded service: {}", name);
    }

    service_ = this->create_service<robo_msgs::srv::ManageService>(
        "manage_service",
        std::bind(&ServiceManager::handle_request, this, std::placeholders::_1,
                  std::placeholders::_2));

    spdlog::info("Service Manager Service Ready");
  }

private:
  void handle_request(
      const std::shared_ptr<robo_msgs::srv::ManageService::Request> request,
      std::shared_ptr<robo_msgs::srv::ManageService::Response> response) {

    std::string service_name = request->service_name;
    std::string command = request->command;

    if (services_.find(service_name) == services_.end()) {
      response->success = false;
      response->message = "Unknown service: " + service_name;
      spdlog::error("{}", response->message);
      return;
    }

    ServiceInfo info = services_[service_name];
    std::string cmd;
    int ret = 0;

    if (command == "start") {
      // now simple backgrounding We use ros2 run
      cmd = "ros2 run " + info.package + " " + info.executable + " " +
            info.params + " > /dev/null 2>&1 &";
      ret = std::system(cmd.c_str());
      if (ret == 0) {
        response->success = true;
        response->message = "Started " + service_name;
      } else {
        response->success = false;
        response->message = "Failed to start " + service_name;
      }
    } else if (command == "stop") {
      // Using pkill on the executable name. This is risky if multiple nodes
      // share the same executable name. But since these seem to be unique
      // executables per service, it might work.
      cmd = "pkill -f " + info.executable; // -f matches full command line
      ret = std::system(cmd.c_str());
      if (ret == 0) {
        response->success = true;
        response->message = "Stopped " + service_name;
      } else {
        response->success = false;
        response->message =
            "Failed to stop " + service_name + " (Process not found?)";
      }
    } else if (command == "restart") {
      // Stop
      std::string stop_cmd = "pkill -f " + info.executable;
      [[maybe_unused]] int stop_ret = std::system(stop_cmd.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for cleanup

      // Start
      cmd = "ros2 run " + info.package + " " + info.executable + " " +
            info.params + " > /dev/null 2>&1 &";
      ret = std::system(cmd.c_str());
      if (ret == 0) {
        response->success = true;
        response->message = "Restarted " + service_name;
      } else {
        response->success = false;
        response->message = "Failed to restart " + service_name;
      }

    } else if (command == "status") {
      // Check if running
      cmd = "pgrep -x " + info.executable + " > /dev/null";
      ret = std::system(cmd.c_str());
      if (ret == 0) {
        response->success = true;
        response->message = "Running";
      } else {
        response->success = false;
        response->message = "Stopped";
      }
    } else {
      response->success = false;
      response->message = "Unknown command: " + command;
    }

    spdlog::info("Request: Service={}, Cmd={} -> {}", service_name, command,
                 response->message);
  }

  std::map<std::string, ServiceInfo> services_;
  rclcpp::Service<robo_msgs::srv::ManageService>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
