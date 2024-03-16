# IMC PlanControlState message in ROS form.
# Header
imc_ros_interface/Header header

std_msgs/UInt8 state
# state enums
# uint8 BLOCKED = 0
# uint8 READY = 1
# uint8 INITIALIZING = 2
# uint8 EXECUTING = 3


std_msgs/String plan_id
std_msgs/Int32 plan_eta
# percent
std_msgs/Float32 plan_progress

std_msgs/String man_id
std_msgs/UInt16 man_type
std_msgs/Int32 man_eta
std_msgs/UInt8 last_outcome

# last outcome enums
# uint8 NONE = 0
# uint8 SUCCESS = 1
# uint8 FAILURE = 2
