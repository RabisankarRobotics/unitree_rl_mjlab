#include <power_dist_board_sdk/power_dist_board.hpp>

#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM0";

  try {
    power_dist_board_sdk::PowerDistBoard pdb(port);
    std::cout << "Listening on " << port << " (Ctrl-C to exit)\n";

    while (true) {
      auto t = pdb.telemetry();
      auto a = pdb.alert();

      std::string alert_msg = a.message();
      if (alert_msg.empty()) alert_msg = "none";

      std::cout << "T=" << t.temperature_c << "C  "
                << "I=" << t.current_ma << "mA  "
                << "V=" << t.voltage_mv << "mV  "
                << "estop=" << (t.estop_pressed ? 1 : 0) << "  "
                << "shdn=" << (t.shutdown_requested ? 1 : 0) << "  "
                << "alerts=0x" << std::hex << std::setw(4) << std::setfill('0')
                << a.flags << std::dec << std::setfill(' ')
                << " [" << alert_msg << "]  "
                << "frames=" << pdb.telemetryCount() << "  "
                << "crc_err=" << pdb.crcErrorCount() << "  "
                << "frm_err=" << pdb.framingErrorCount() << "\n";

      ::usleep(200000);  // 5 Hz
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
