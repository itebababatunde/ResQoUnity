# DroneState.msg - Quadcopter drone state information

# Altitude above ground (meters)
float32 altitude

# Battery level (0.0 to 1.0, where 1.0 is fully charged)
float32 battery_level

# Motor speeds (RPM or normalized 0-1 for each of 4 motors)
float32[4] motor_speeds

# Flight mode (0=manual, 1=stabilized, 2=autonomous)
uint8 flight_mode

# Armed state (true if motors can spin)
bool armed

# GPS coordinates (if available)
float64 latitude
float64 longitude

# Estimated velocity (m/s) in body frame
float32[3] velocity_body

# Barometric pressure (Pa)
float32 pressure

