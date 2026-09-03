std_msgs/Header header

# NORMAL, GPS_BLACKOUT, GPS_NOISE, IMU_NOISE, SENSOR_DEGRADED
string state
bool gps_valid
bool gps_blackout
bool gps_noisy
bool imu_noisy
bool gps_recovering
bool imu_stale

float32 gps_residual_m
float32 gps_step_speed_mps
float32 imu_gyro_spike_score
float32 imu_accel_spike_score
float32 confidence
string reason
