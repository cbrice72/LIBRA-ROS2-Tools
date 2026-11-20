# LIBRA App runtime diagnostics

std_msgs/Header header

float64 power_hebi           # V*I (W)
float64 power_epos           # I^2*R + k_t*I*omega (W)
float64 power_fluid          # valves + pump, from commanded state (W)
float64 torque_actual        # measured holding torque, tau_actuator (Nm)
float64 torque_model_actual  # predicted holding torque at actual angle, -tau_ext,g(theta) from Eq. 36' (Nm)
float64 torque_model_target  # predicted holding torque at target angle, -tau_ext,g(theta_tgt) from Eq. 39' (Nm)
