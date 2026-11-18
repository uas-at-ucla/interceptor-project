# Target estimate from the Tracker, published on /target_data.
# Published only once a track exists, then at a fixed rate, including
# predicted updates between camera detections.

std_msgs/Header header                 # stamp: time of this estimate; frame_id: same frame as /mavros/local_position/pose
geometry_msgs/Point position           # m, local ENU
geometry_msgs/Vector3 velocity         # m/s, local ENU
builtin_interfaces/Time last_seen      # time of the last real camera detection; never updated by predictions
