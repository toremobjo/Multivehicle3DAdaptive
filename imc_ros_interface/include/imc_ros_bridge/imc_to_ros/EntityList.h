// LICENCE

#ifndef IMC_TO_ROS_ENTITYLIST_H
#define IMC_TO_ROS_ENTITYLIST_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/EntityList.h>
#include <IMC/Spec/EntityList.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::EntityList& imc_msg, imc_ros_interface::EntityList& ros_msg);

}

#endif
