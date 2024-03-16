# The IMC identifier of the source system that is allowed to control the vehicle.
# If the value ''0xFFFF'' is used, any system is allowed to command references.
imc_ros_interface/Header header
std_msgs/UInt16 control_src
std_msgs/UInt8 control_ent
imc_ros_interface/Reference reference
std_msgs/UInt8 state
std_msgs/UInt8 proximity
