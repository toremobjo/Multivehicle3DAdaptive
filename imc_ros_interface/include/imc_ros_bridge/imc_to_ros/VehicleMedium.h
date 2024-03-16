// LICENCE

#ifndef IMC_TO_ROS_VEHICLEMEDIUM_H
#define IMC_TO_ROS_VEHICLEMEDIUM_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/VehicleMedium.h>
#include <IMC/Spec/VehicleMedium.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::VehicleMedium& imc_msg, imc_ros_interface::VehicleMedium& ros_msg);

}

#endif
