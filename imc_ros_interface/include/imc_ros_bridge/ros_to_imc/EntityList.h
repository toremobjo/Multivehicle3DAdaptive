//Licence
#ifndef ROS_TO_IMC_ENTITYLIST_H
#define ROS_TO_IMC_ENTITYLIST_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/EntityList.h"
#include <IMC/Spec/EntityList.hpp>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::EntityList& ros_msg, IMC::EntityList& imc_msg);

} // namespace ros_to_imc
#endif // ROS_TO_IMC_DESIREDHEADING_H
