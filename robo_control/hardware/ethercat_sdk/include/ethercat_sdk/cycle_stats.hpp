#pragma once

#include <cstdint>

namespace ethercat_sdk {

/// Per-cycle timing snapshot produced by rtLoop.
/// All timestamps are in nanoseconds from steady_clock epoch.
/// Consumers compute intervals by subtracting pairs of timestamps.
struct CycleTimingSnapshot {
  int64_t intended_wakeup_ns;   // When we asked OS to wake us
  int64_t wakeup_ns;            // Actual wakeup (= cycle start)
  int64_t after_receive_ns;     // After ecx_receive_processdata()
  int64_t after_txpdo_read_ns;  // After reading all TxPDO feedback
  int64_t after_rxpdo_write_ns; // After writing all RxPDO commands
  int64_t after_send_ns;        // After ecx_send_processdata()
  int64_t after_dc_sync_ns;     // After dcSync()
  int64_t cycle_end_ns;         // After mailbox handler
  int64_t dc_offset_ns;         // toff value from DC PI controller
  int wkc;                      // Working counter this cycle
  int expected_wkc;             // Expected working counter
  uint64_t cycle_count;         // Monotonic cycle counter
};

} // namespace ethercat_sdk
