// LICENCE

#ifndef IMC_TO_ROS_VEHICLESTATE_H
#define IMC_TO_ROS_VEHICLESTATE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/VehicleState.h>
#include <IMC/Spec/VehicleState.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::VehicleState& imc_msg, imc_ros_interface::VehicleState& ros_msg);

}

#endif
