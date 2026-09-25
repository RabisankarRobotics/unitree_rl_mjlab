#pragma once

#include "transmission_sdk/ankle_transmission_tahiti_c1.hpp"
#include "transmission_sdk/transmission_base.hpp"

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace transmission {

// Factory: build a concrete transmission for the given robot type and
// transmission kind. Throws std::runtime_error if the combination is unknown.
inline std::unique_ptr<TransmissionBase> createTransmission(
    const std::string& robot_type, const std::string& tx_type) {
  if (robot_type == "tahiti_c1" && tx_type == "ankle") {
    return std::make_unique<AnkleTransmissionTahitiC1>();
  }
  std::ostringstream os;
  os << "Unknown transmission (robot_type=\"" << robot_type
     << "\", type=\"" << tx_type << "\")";
  throw std::runtime_error(os.str());
}

}  // namespace transmission
