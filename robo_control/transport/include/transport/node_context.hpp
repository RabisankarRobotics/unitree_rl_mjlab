#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace transport {

/**
 * @brief Singleton managing shared ROS2 node and executor.
 *
 * Provides a shared rclcpp::Node that all publishers and subscribers
 * in the process can use. Runs a MultiThreadedExecutor in a background
 * thread to process callbacks.
 */
class NodeContext {
public:
  /**
   * @brief Get the singleton instance.
   */
  static NodeContext &instance() {
    static NodeContext instance;
    return instance;
  }

  /**
   * @brief Initialize ROS2 context and create the shared node.
   *
   * @param node_name Name for the ROS2 node
   * @param argc Command line argument count
   * @param argv Command line arguments
   */
  void init(const std::string &node_name, int argc = 0, char **argv = nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
      return;
    }

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node with options
    rclcpp::NodeOptions options;
    node_ = std::make_shared<rclcpp::Node>(node_name, options);

    // Create executor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    // Start executor in background thread
    running_.store(true);
    executor_thread_ = std::thread([this]() {
      while (running_.load()) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });

    initialized_ = true;
  }

  /**
   * @brief Get the shared ROS2 node.
   *
   * @return Shared pointer to the node
   */
  std::shared_ptr<rclcpp::Node> node() {
    std::lock_guard<std::mutex> lock(mutex_);
    return node_;
  }

  /**
   * @brief Check if the context is initialized and ROS2 is running.
   */
  bool ok() const { return initialized_ && rclcpp::ok(); }

  /**
   * @brief Clean shutdown of ROS2 context.
   */
  void shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
      return;
    }

    // Stop executor thread
    running_.store(false);
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    // Clean up executor and node
    if (executor_) {
      executor_->cancel();
      executor_.reset();
    }
    node_.reset();

    // Shutdown ROS2
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    initialized_ = false;
  }

  // Delete copy/move operations
  NodeContext(const NodeContext &) = delete;
  NodeContext &operator=(const NodeContext &) = delete;
  NodeContext(NodeContext &&) = delete;
  NodeContext &operator=(NodeContext &&) = delete;

private:
  NodeContext() = default;
  ~NodeContext() { shutdown(); }

  std::mutex mutex_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<bool> running_{false};
  bool initialized_{false};
};
} // namespace transport
