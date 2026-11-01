geometry_msgs/Transform base_transform  # Base pose in the map frame

geometry_msgs/Transform sensor_transform  # Sensor pose in the map frame; its x axis is the viewing direction

float64[] joint_position  # Joint positions of the robot, any length; empty for the shipped robot models

duration time_from_start  # Time of this state within its trajectory; 0 for the first point
