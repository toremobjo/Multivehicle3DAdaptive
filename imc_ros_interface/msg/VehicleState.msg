# IMC VehicleState message in ROS form.
#      This message summarizes the overall state of the vehicle. It can
#      contains information regarding:
#
#      - The overall operation mode.
#      - Any error conditions.
#      - Current maneuver execution.
#      - Active control loops.

#Header
imc_ros_interface/Header header

#enums, op mode
#std_msgs/UInt8 SERVICE       = 0
#std_msgs/UInt8 CALIBRATION   = 1
#std_msgs/UInt8 ERROR         = 2
#std_msgs/UInt8 MANEUVER      = 3
#std_msgs/UInt8 EXTERNAL      = 4
#std_msgs/UInt8 BOOT          = 5


# fields
std_msgs/UInt8 op_mode           # Operation Mode.
std_msgs/UInt8 error_count       # Errors   -- Count.
std_msgs/String error_ents       # Errors   -- Entities.
std_msgs/UInt16 maneuver_type    # Maneuver -- Type.
std_msgs/Float64 maneuver_stime  # Maneuver -- Start Time.
std_msgs/UInt16 maneuver_eta     # Maneuver -- ETA.
std_msgs/UInt32 control_loops    # Control Loops.
std_msgs/UInt8 flags             # Flags.
std_msgs/String last_error       # Last Error -- Description.
std_msgs/Float64 last_error_time # Last Error -- Time.
