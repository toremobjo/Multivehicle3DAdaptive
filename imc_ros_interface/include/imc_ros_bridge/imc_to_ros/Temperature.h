// LICENCE

#ifndef IMC_TO_ROS_TEMPERATURE_H
#define IMC_TO_ROS_TEMPERATURE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/Temperature.h>
#include <IMC/Spec/Temperature.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Temperature& imc_msg, imc_ros_interface::Temperature& ros_msg);

}

#endif
