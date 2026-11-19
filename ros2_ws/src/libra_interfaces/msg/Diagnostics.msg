# LIBRA App runtime diagnostics

std_msgs/Header header

float64 power_hebi    # V*I (W)
float64 power_epos    # I^2*R + k_t*I*omega (W)
float64 power_fluid   # valves + pump, from commanded state (W)
float64 torque_meas   # actual, tau_meas = -tau_actuator (Nm)
float64 torque_ext_g  # predicted, tau_ext,g(theta) from Eq. 36' (Nm)
