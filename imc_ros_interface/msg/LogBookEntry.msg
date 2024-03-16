#Human readable message reporting an event of interest.
imc_ros_interface/Header header
std_msgs/UInt8 type #0=INFO,1=warning,2=error,3=critical,4=debug
std_msgs/Float64 htime
std_msgs/String context
std_msgs/String text
