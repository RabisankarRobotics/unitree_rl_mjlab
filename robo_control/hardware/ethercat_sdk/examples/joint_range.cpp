#include "ethercat_sdk/master.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

static volatile bool running = true;
static void sighandler(int) { running = false; }

struct RangeTracker {
  bool seen = false;
  double min_rad = 0.0;
  double max_rad = 0.0;
  double cur_rad = 0.0;

  void update(double p) {
    cur_rad = p;
    if (!seen) {
      min_rad = max_rad = p;
      seen = true;
    } else {
      if (p < min_rad) min_rad = p;
      if (p > max_rad) max_rad = p;
    }
  }
  void reset() {
    seen = false;
    min_rad = max_rad = cur_rad = 0.0;
  }
};

// Non-blocking single-char read from stdin without echo.
class RawStdin {
public:
  RawStdin() {
    if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
      saved_ = true;
      termios raw = orig_;
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
  }
  ~RawStdin() {
    if (saved_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }
  int poll_char() {
    unsigned char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    return n == 1 ? int(c) : -1;
  }
private:
  termios orig_{};
  bool saved_ = false;
};

int main(int argc, char *argv[]) {
  std::signal(SIGINT, sighandler);

  std::string interface = "enP3p49s0";
  int expected_count = 12;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--count") == 0 && i + 1 < argc) {
      expected_count = std::atoi(argv[++i]);
    } else {
      interface = argv[i];
    }
  }

  ethercat_sdk::EtherCATMaster master(interface);

  std::map<int, std::string> type_map;
  for (int bus_id = 1; bus_id <= expected_count; ++bus_id) {
    type_map[bus_id] = "X8-120";
  }
  master.configActuatorTypes(type_map);

  if (!master.init()) {
    std::cerr << "Failed to initialize EtherCAT on " << interface << std::endl;
    return 1;
  }

  const int slave_count = master.getSlaveCount();
  if (slave_count == 0) {
    std::cerr << "No slaves found" << std::endl;
    return 1;
  }

  std::cout << "Found " << slave_count << " slave(s) on " << interface << "\n";
  std::cout << "Motors stay DISABLED - manually backdrive each joint through "
               "its full range.\n";
  std::cout << "Keys: [r] reset all  [1-9,0=10,a=11,b=12] reset one  [q] quit\n\n";

  master.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::vector<RangeTracker> trk(slave_count + 1); // 1-indexed

  RawStdin stdin_raw;
  std::cout << std::fixed << std::setprecision(3);

  const int header_lines = 1;
  bool first = true;

  auto bus_id_for_key = [&](int c) -> int {
    if (c >= '1' && c <= '9') return c - '0';
    if (c == '0') return 10;
    if (c == 'a' || c == 'A') return 11;
    if (c == 'b' || c == 'B') return 12;
    return -1;
  };

  while (running) {
    int c;
    while ((c = stdin_raw.poll_char()) != -1) {
      if (c == 'q' || c == 'Q') {
        running = false;
      } else if (c == 'r' || c == 'R') {
        for (auto &t : trk) t.reset();
      } else {
        int id = bus_id_for_key(c);
        if (id >= 1 && id <= slave_count) trk[id].reset();
      }
    }

    if (!first) {
      std::cout << "\033[" << (slave_count + header_lines) << "A";
    }
    first = false;

    std::cout
        << "  slave |   current   |     min     |     max     |    range\033[K\n";
    for (int bus_id = 1; bus_id <= slave_count; ++bus_id) {
      auto state = master.getActuatorState(bus_id);
      trk[bus_id].update(state.position);

      const double r2d = 180.0 / M_PI;
      double cur = trk[bus_id].cur_rad * r2d;
      double mn = trk[bus_id].min_rad * r2d;
      double mx = trk[bus_id].max_rad * r2d;
      double rng = (trk[bus_id].max_rad - trk[bus_id].min_rad) * r2d;

      std::cout << "    " << std::setw(2) << bus_id << "  | "
                << std::setw(9) << cur << "   | "
                << std::setw(9) << mn << "   | "
                << std::setw(9) << mx << "   | "
                << std::setw(9) << rng << " deg\033[K\n";
    }
    std::cout << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  master.stop();

  std::cout << "\n=== Final ranges ===\n";
  std::cout << "  slave |        min (deg / rad)         |        max (deg "
               "/ rad)         |     range (deg / rad)\n";
  for (int bus_id = 1; bus_id <= slave_count; ++bus_id) {
    const auto &t = trk[bus_id];
    if (!t.seen) continue;
    const double r2d = 180.0 / M_PI;
    double mn_d = t.min_rad * r2d, mx_d = t.max_rad * r2d;
    double rng_d = (t.max_rad - t.min_rad) * r2d;
    std::cout << "    " << std::setw(2) << bus_id << "  | "
              << std::setw(9) << mn_d << " / " << std::setw(8) << t.min_rad
              << "  | "
              << std::setw(9) << mx_d << " / " << std::setw(8) << t.max_rad
              << "  | "
              << std::setw(9) << rng_d << " / " << std::setw(8)
              << (t.max_rad - t.min_rad) << "\n";
  }

  return 0;
}
