// LICENCE

#ifndef IMC_TO_ROS_PLANCONTROLSTATE_H
#define IMC_TO_ROS_PLANCONTROLSTATE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/PlanControlState.h>
#include <IMC/Spec/PlanControlState.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanControlState& imc_msg, imc_ros_interface::PlanControlState& ros_msg);

}

#endif
