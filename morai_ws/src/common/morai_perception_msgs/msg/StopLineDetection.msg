std_msgs/Header header

# header.stamp: source frame time in the ROS clock domain, NOT inference end.
# UDP mode uses local completed-frame receipt time (not simulator capture time).
# header.frame_id: BEV ego frame (base_link by default).
# distance_m: forward distance from BEV ego origin; controller subtracts
# front_reference_offset_m to locate the front-wheel/front-bumper reference.
# confidence: geometric support from transverse width, coverage and thickness;
# not a calibrated probability or the quality of the surrounding lane boundaries.
bool valid
float64 distance_m
float64 confidence
