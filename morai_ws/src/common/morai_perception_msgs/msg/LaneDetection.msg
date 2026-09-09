std_msgs/Header header
# Body-relative errors: x forward, y left; header stamp is source observation time.
# lateral_offset_m = -lane_center_y at the producer's reference distance.
# Positive lateral offset means lane centre is to the RIGHT; steer right to correct.
# Positive heading error means lane direction is to the LEFT of vehicle heading.
# ROI producer reference: lateral at 7 m, heading from the 7--14 m centre chord.
float32 lateral_offset_m
float32 heading_error_rad
float32 confidence
bool valid
