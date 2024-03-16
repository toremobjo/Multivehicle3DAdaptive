// LICENCE

#ifndef IMC_TO_ROS_FOLLOWREFSTATE_H
#define IMC_TO_ROS_FOLLOWREFSTATE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/FollowRefState.h>
#include <IMC/Spec/FollowRefState.hpp>
#include <IMC/Spec/Reference.hpp>
#include <IMC/Spec/DesiredZ.hpp>
#include <IMC/Spec/DesiredSpeed.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::FollowRefState& imc_msg, imc_ros_interface::FollowRefState& ros_msg);

}

#endif
