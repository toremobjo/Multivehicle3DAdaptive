// LICENCE
#ifndef ROS_TO_IMC_SMS
#define ROS_TO_IMC_SMS

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/Sms.h"
#include <IMC/Spec/Sms.hpp>


namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::Sms& ros_msg, IMC::Sms& imc_msg);

} // namespace ros_to_imc
#endif
