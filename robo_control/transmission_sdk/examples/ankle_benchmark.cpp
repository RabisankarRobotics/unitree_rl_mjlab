#include "transmission_sdk/ankle_transmission_tahiti_c1.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

// Microbenchmark for the Tahiti C1 ankle transmission.
//
// Each operation is timed in isolation over N_ITERS iterations across a grid of
// joint configurations. Reports avg / min / max per call in nanoseconds.
// Build: colcon build --packages-select transmission_sdk
// Run:   ./install/transmission_sdk/lib/transmission_sdk/ankle_benchmark

namespace {

using transmission::AnkleTransmissionTahitiC1;
using Clock = std::chrono::steady_clock;

struct Stats {
  uint64_t sum_ns = 0;
  uint64_t min_ns = UINT64_MAX;
  uint64_t max_ns = 0;
  uint64_t count = 0;

  void add(uint64_t ns) {
    sum_ns += ns;
    if (ns < min_ns) min_ns = ns;
    if (ns > max_ns) max_ns = ns;
    ++count;
  }

  void print(const char* name) const {
    double avg = static_cast<double>(sum_ns) / static_cast<double>(count);
    std::printf("  %-22s %10.1f / %8lu / %10lu   (%lu calls)\n", name, avg,
                static_cast<unsigned long>(min_ns),
                static_cast<unsigned long>(max_ns),
                static_cast<unsigned long>(count));
  }
};

// Force Eigen to actually materialise results, defeating dead-store elimination.
volatile double sink = 0.0;
void consume(const Eigen::VectorXd& v) {
  sink += v(0) + v(1);
}

}  // namespace

int main(int argc, char** argv) {
  AnkleTransmissionTahitiC1 tx;

  const int n_iters = (argc > 1) ? std::atoi(argv[1]) : 20000;

  // Pre-generate a sweep of feasible joint configurations. Using the same
  // 0.22 rad window as ankle_roundtrip to stay inside the workspace.
  constexpr double kRange = 0.22;
  std::mt19937 rng(0xC0FFEE);
  std::uniform_real_distribution<double> dist(-kRange, kRange);

  std::vector<Eigen::VectorXd> q_joints(n_iters, Eigen::VectorXd(2));
  std::vector<Eigen::VectorXd> q_motors(n_iters, Eigen::VectorXd(2));
  std::vector<Eigen::VectorXd> dq_joints(n_iters, Eigen::VectorXd(2));
  std::vector<Eigen::VectorXd> dq_motors(n_iters, Eigen::VectorXd(2));
  std::vector<Eigen::VectorXd> tau_joints(n_iters, Eigen::VectorXd(2));
  std::vector<Eigen::VectorXd> tau_motors(n_iters, Eigen::VectorXd(2));

  for (int i = 0; i < n_iters; ++i) {
    q_joints[i] << dist(rng), dist(rng);
    // Pre-compute motor-space truth via forward, so inverse has a valid target.
    q_motors[i] = tx.position_forward(q_joints[i]);
    dq_joints[i] << dist(rng), dist(rng);
    dq_motors[i] = tx.velocity_forward(q_joints[i], dq_joints[i]);
    tau_joints[i] << dist(rng), dist(rng);
    tau_motors[i] = tx.torque_forward(q_joints[i], tau_joints[i]);
  }

  Eigen::VectorXd out(2);

  // Warm up: touch the code paths so the instruction cache and branch
  // predictors are in steady state before we start measuring.
  for (int i = 0; i < 500; ++i) {
    tx.position_forward_into(q_joints[i % n_iters], out);
    tx.velocity_forward_into(q_joints[i % n_iters],
                             dq_joints[i % n_iters], out);
    tx.torque_forward_into(q_joints[i % n_iters],
                           tau_joints[i % n_iters], out);
    tx.position_inverse_into(q_motors[i % n_iters], q_joints[i % n_iters], out);
    tx.velocity_inverse_into(q_joints[i % n_iters],
                             dq_motors[i % n_iters], out);
    tx.torque_inverse_into(q_joints[i % n_iters],
                           tau_motors[i % n_iters], out);
    consume(out);
  }

  Stats s_pos_fwd, s_vel_fwd, s_trq_fwd;
  Stats s_pos_inv_cold, s_pos_inv_warm;
  Stats s_vel_inv, s_trq_inv;
  Stats s_fwd_triple, s_inv_triple, s_full_cycle;

  // Individual operations.
  for (int i = 0; i < n_iters; ++i) {
    {
      auto t0 = Clock::now();
      tx.position_forward_into(q_joints[i], out);
      auto t1 = Clock::now();
      s_pos_fwd.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        t1 - t0).count());
      consume(out);
    }
    {
      auto t0 = Clock::now();
      tx.velocity_forward_into(q_joints[i], dq_joints[i], out);
      auto t1 = Clock::now();
      s_vel_fwd.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        t1 - t0).count());
      consume(out);
    }
    {
      auto t0 = Clock::now();
      tx.torque_forward_into(q_joints[i], tau_joints[i], out);
      auto t1 = Clock::now();
      s_trq_fwd.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        t1 - t0).count());
      consume(out);
    }
    {
      // Cold seed: the seed is far from the solution.
      Eigen::VectorXd seed = Eigen::VectorXd::Zero(2);
      auto t0 = Clock::now();
      tx.position_inverse_into(q_motors[i], seed, out);
      auto t1 = Clock::now();
      s_pos_inv_cold.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                             t1 - t0).count());
      consume(out);
    }
    {
      // Warm seed: the seed is the true solution (how control_service actually
      // uses it — the previous cycle's converged q_joint).
      auto t0 = Clock::now();
      tx.position_inverse_into(q_motors[i], q_joints[i], out);
      auto t1 = Clock::now();
      s_pos_inv_warm.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                             t1 - t0).count());
      consume(out);
    }
    {
      auto t0 = Clock::now();
      tx.velocity_inverse_into(q_joints[i], dq_motors[i], out);
      auto t1 = Clock::now();
      s_vel_inv.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        t1 - t0).count());
      consume(out);
    }
    {
      auto t0 = Clock::now();
      tx.torque_inverse_into(q_joints[i], tau_motors[i], out);
      auto t1 = Clock::now();
      s_trq_inv.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        t1 - t0).count());
      consume(out);
    }
  }

  // Composite: what control_service actually does per cycle, per group.
  Eigen::VectorXd out_q(2), out_dq(2), out_tau(2);
  for (int i = 0; i < n_iters; ++i) {
    {
      auto t0 = Clock::now();
      tx.position_forward_into(q_joints[i], out_q);
      tx.velocity_forward_into(q_joints[i], dq_joints[i], out_dq);
      tx.torque_forward_into(q_joints[i], tau_joints[i], out_tau);
      auto t1 = Clock::now();
      s_fwd_triple.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           t1 - t0).count());
      consume(out_q); consume(out_dq); consume(out_tau);
    }
    {
      auto t0 = Clock::now();
      tx.position_inverse_into(q_motors[i], q_joints[i], out_q);
      tx.velocity_inverse_into(out_q, dq_motors[i], out_dq);
      tx.torque_inverse_into(out_q, tau_motors[i], out_tau);
      auto t1 = Clock::now();
      s_inv_triple.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           t1 - t0).count());
      consume(out_q); consume(out_dq); consume(out_tau);
    }
    {
      // What one ankle group costs per control cycle (inv + fwd, three ops each).
      auto t0 = Clock::now();
      tx.position_inverse_into(q_motors[i], q_joints[i], out_q);
      tx.velocity_inverse_into(out_q, dq_motors[i], out_dq);
      tx.torque_inverse_into(out_q, tau_motors[i], out_tau);
      tx.position_forward_into(q_joints[i], out_q);
      tx.velocity_forward_into(q_joints[i], dq_joints[i], out_dq);
      tx.torque_forward_into(q_joints[i], tau_joints[i], out_tau);
      auto t1 = Clock::now();
      s_full_cycle.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           t1 - t0).count());
      consume(out_q); consume(out_dq); consume(out_tau);
    }
  }

  std::printf("AnkleTransmissionTahitiC1 microbenchmark, %d iterations\n",
              n_iters);
  std::printf("  %-22s %10s / %8s / %10s\n", "op", "avg_ns", "min_ns",
              "max_ns");
  s_pos_fwd.print("position_forward");
  s_vel_fwd.print("velocity_forward");
  s_trq_fwd.print("torque_forward");
  s_pos_inv_cold.print("position_inverse cold");
  s_pos_inv_warm.print("position_inverse warm");
  s_vel_inv.print("velocity_inverse");
  s_trq_inv.print("torque_inverse");
  std::printf("  --- composite (one ankle group) ---\n");
  s_fwd_triple.print("fwd triple (pos+vel+trq)");
  s_inv_triple.print("inv triple (pos+vel+trq)");
  s_full_cycle.print("full cycle (inv+fwd)");
  std::printf("  * full cycle x2 groups = per-cycle transmission cost.\n");

  // Return non-zero sink so aggressive optimisers can't DCE.
  return sink == 42.0 ? 1 : 0;
}
