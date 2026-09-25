#pragma once

#include "node_context.hpp"
#include "types.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace transport {

template <typename T> class Subscriber {
public:
  using CallbackType = std::function<void(const T &)>;

  explicit Subscriber(const std::string &topic_name,
                      uint32_t /*domain_id*/ = DEFAULT_DOMAIN_ID) {
    auto node = NodeContext::instance().node();
    if (!node) {
      std::cerr << "NodeContext not initialized - call NodeContext::init() "
                   "before creating subscribers"
                << std::endl;
      return;
    }

    try {
      // Use default QoS: KeepLast(10), Reliable, Volatile
      subscription_ = node->create_subscription<T>(
          topic_name, rclcpp::QoS(10),
          [this](const typename T::SharedPtr msg) {
            if (callback_) {
              callback_(*msg);
            }
          });
    } catch (const std::exception &e) {
      std::cerr << "ROS2 Subscriber initialization error: " << e.what()
                << std::endl;
    }
  }

  ~Subscriber() = default;

  // Delete copy constructor and assignment
  Subscriber(const Subscriber &) = delete;
  Subscriber &operator=(const Subscriber &) = delete;

  // Allow move semantics
  Subscriber(Subscriber &&) noexcept = default;
  Subscriber &operator=(Subscriber &&) noexcept = default;

  void set_callback(CallbackType callback) { callback_ = std::move(callback); }

  bool is_valid() const { return subscription_ != nullptr; }

private:
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  CallbackType callback_;
};

} // namespace transport
