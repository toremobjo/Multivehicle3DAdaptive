//LICENCE

#ifndef ROS_TO_IMC_LOGBOOKENTRY_H
#define ROS_TO_IMC_LOGBOOKENTRY_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/LogBookEntry.h"
#include <IMC/Spec/LogBookEntry.hpp>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::LogBookEntry& ros_msg, IMC::LogBookEntry& imc_msg);

} // namespace ros_to_imc
#endif
