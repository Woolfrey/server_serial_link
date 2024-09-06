float64 time                            # Point in time at which to reach the state
geometry_msgs/Pose   pose               # Desired position and orientation
geometry_msgs/Twist  twist              # Desired linear velocity (m/s) and angular velocity (rad/s)
geometry_msgs/Accel  accel              # Desired linear velocity (m/s/s) and angular acceleration (rad/s/s)
int32 GLOBAL   = 0                      # In the global coordinate system
int32 LOCAL    = 1                      # In the local coordinate frame of the endpoint
int32 RELATIVE = 2                      # Relative to endpoint frame, but in global coordinates
int32 reference
