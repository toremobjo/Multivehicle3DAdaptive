// LICENCE

#ifndef IMC_TO_ROS_ABORT_H
#define IMC_TO_ROS_ABORT_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/Abort.h>
#include <IMC/Spec/Abort.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Abort& imc_msg, imc_ros_interface::Abort& ros_msg);

}

#endif
