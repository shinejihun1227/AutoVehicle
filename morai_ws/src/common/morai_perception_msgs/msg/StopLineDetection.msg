std_msgs/Header header

# header.stamp: source frame time in the ROS clock domain, NOT inference end.
# UDP mode uses local completed-frame receipt time (not simulator capture time).
# header.frame_id: BEV ego frame (base_link by default).
# distance_m: forward distance from BEV ego origin; controller subtracts
# front_reference_offset_m to locate the front-wheel/front-bumper reference.
# confidence: binary geometric validity proxy in current detector, not probability.
bool valid
float64 distance_m
float64 confidence
