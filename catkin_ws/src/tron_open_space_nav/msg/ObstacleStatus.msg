std_msgs/Header header

string state

# LiDAR/base-frame origin to the nearest front obstacle point.
float32 raw_front_distance
# Inflated front-footprint edge to the nearest front obstacle point.
float32 front_distance
float32 left_clearance
float32 right_clearance

uint32 front_points
uint32 left_points
uint32 right_points

float32 front_occupancy
float32 left_occupancy
float32 right_occupancy

float32 pointcloud_age
float32 state_age
float32 selected_direction_age

uint32 raw_points
uint32 after_tf_points
uint32 after_self_filter_points
uint32 after_roi_points
uint32 after_ground_points

float32 min_x
float32 max_x
float32 min_y
float32 max_y
float32 min_z
float32 max_z

bool obstacle_detected
bool fail_safe_active
