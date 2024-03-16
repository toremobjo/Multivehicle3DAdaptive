// LICENCE
#ifndef ROS_TO_IMC_REFERENCE_H
#define ROS_TO_IMC_REFERENCE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/Reference.h"
#include <IMC/Spec/Reference.hpp>
#include <IMC/Spec/DesiredZ.hpp>
#include <IMC/Spec/DesiredSpeed.hpp>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::Reference& ros_msg, IMC::Reference& imc_msg);

} // namespace ros_to_imc
#endif 
