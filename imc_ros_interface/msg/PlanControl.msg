# IMC PlanControl message in ROS form. msg no 559

# enums
std_msgs/UInt8 type #0,1,2,3 -> request, success, failure, in progress
std_msgs/UInt8 op # 0,1,2,3 -> start, stop, load, get

# fields
std_msgs/UInt16 request_id
std_msgs/String plan_id
std_msgs/UInt16 flags
std_msgs/String info

std_msgs/UInt16 source
