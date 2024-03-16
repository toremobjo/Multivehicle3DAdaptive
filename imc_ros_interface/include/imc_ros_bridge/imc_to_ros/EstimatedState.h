// LICENCE

#ifndef IMC_TO_ROS_ESTIMATEDSTATE_H
#define IMC_TO_ROS_ESTIMATEDSTATE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/EstimatedState.h>
#include <IMC/Spec/EstimatedState.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::EstimatedState& imc_msg, imc_ros_interface::EstimatedState& ros_msg);

}

#endif
