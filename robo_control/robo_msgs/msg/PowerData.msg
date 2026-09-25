int32 temperature_c          # Board temperature (degrees C)
uint16 current_ma            # Bus current (mA)
uint16 voltage_mv            # Bus voltage (mV)
bool estop_pressed           # E-stop button pressed
bool shutdown_requested      # Shutdown requested by board
uint16 alert_flags           # Bitmask of active alerts (see PowerDistBoard AlertFlag)
string alert_message         # Human-readable comma-separated list of active alert names
