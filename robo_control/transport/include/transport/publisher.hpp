#pragma once

#include "node_context.hpp"
#include "types.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace transport {

template <typename T> class Publisher {
public:
  explicit Publisher(const std::string &topic_name,
                     uint32_t /*domain_id*/ = DEFAULT_DOMAIN_ID) {
    auto node = NodeContext::instance().node();
    if (!node) {
      std::cerr << "NodeContext not initialized - call NodeContext::init() "
                   "before creating publishers"
                << std::endl;
      return;
    }

    try {
      // Use default QoS: KeepLast(10), Reliable, Volatile
      publisher_ = node->create_publisher<T>(topic_name, rclcpp::QoS(10));
    } catch (const std::exception &e) {
      std::cerr << "ROS2 Publisher initialization error: " << e.what()
                << std::endl;
    }
  }

  ~Publisher() = default;

  // Delete copy constructor and assignment
  Publisher(const Publisher &) = delete;
  Publisher &operator=(const Publisher &) = delete;

  // Allow move semantics
  Publisher(Publisher &&) noexcept = default;
  Publisher &operator=(Publisher &&) noexcept = default;

  bool publish(const T &msg) {
    if (!is_valid()) {
      std::cerr << "Publisher is not valid" << std::endl;
      return false;
    }

    try {
      publisher_->publish(msg);
      return true;
    } catch (const std::exception &e) {
      std::cerr << "ROS2 publish error: " << e.what() << std::endl;
      return false;
    }
  }

  bool is_valid() const { return publisher_ != nullptr; }

private:
  typename rclcpp::Publisher<T>::SharedPtr publisher_;
};

} // namespace transport
