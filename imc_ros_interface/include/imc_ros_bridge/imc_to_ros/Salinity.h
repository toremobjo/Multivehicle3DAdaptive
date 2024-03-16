// LICENCE

#ifndef IMC_TO_ROS_SALINITY_H
#define IMC_TO_ROS_SALINITY_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/Salinity.h>
#include <IMC/Spec/Salinity.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Salinity& imc_msg, imc_ros_interface::Salinity& ros_msg);

}

#endif
