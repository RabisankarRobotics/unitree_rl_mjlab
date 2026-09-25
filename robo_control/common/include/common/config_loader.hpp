#pragma once

#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace common {

/**
 * @brief Get a required value from a YAML node.
 * Throws a runtime_error if the key is missing or conversion fails.
 *
 * @tparam T Type of the value to retrieve
 * @param node The YAML node to search in
 * @param key The key to look for
 * @return The value of type T
 */
template <typename T>
T get_config_value(const YAML::Node &node, const std::string &key) {
  if (!node[key]) {
    throw std::runtime_error("Missing required config parameter: " + key);
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::BadConversion &e) {
    throw std::runtime_error("Failed to convert config parameter '" + key +
                             "' to required type: " + e.msg);
  }
}

/**
 * @brief Get an optional value from a YAML node.
 * Returns the default value if the key is missing.
 *
 * @tparam T Type of the value to retrieve
 * @param node The YAML node to search in
 * @param key The key to look for
 * @param default_value Value to return if key is missing
 * @return The value of type T
 */
template <typename T>
T get_config_value(const YAML::Node &node, const std::string &key,
                   const T &default_value) {
  if (!node[key]) {
    return default_value;
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::BadConversion &e) {
    spdlog::warn("Failed to convert config parameter '{}', using default: {}",
                 key, e.msg);
    return default_value;
  }
}

} // namespace common
