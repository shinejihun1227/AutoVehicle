std_msgs/Header header

# NORMAL, GPS_BLACKOUT, SENSOR_DEGRADED
string state
bool gps_valid
bool gps_blackout
bool gps_recovering
bool imu_stale

float32 confidence
string reason
