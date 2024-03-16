// LICENCE

#ifndef IMC_TO_ROS_DEPTH_H
#define IMC_TO_ROS_DEPTH_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/Depth.h>
#include <IMC/Spec/Depth.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Depth& imc_msg, imc_ros_interface::Depth& ros_msg);

}

#endif
