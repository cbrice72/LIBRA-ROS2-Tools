# LIBRA counterweight state

std_msgs/Header header

float64 volume         # V (L)
float64 flow           # V', positive = filling (L/s)
float64 target_volume  # V_bal(theta_tgt), predictive CW target (L)
