// LICENCE

#ifndef IMC_TO_ROS_CHLOROPHYLL_H
#define IMC_TO_ROS_CHLOROPHYLL_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/Chlorophyll.h>
#include <IMC/Spec/Chlorophyll.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Chlorophyll& imc_msg, imc_ros_interface::Chlorophyll& ros_msg);

}

#endif