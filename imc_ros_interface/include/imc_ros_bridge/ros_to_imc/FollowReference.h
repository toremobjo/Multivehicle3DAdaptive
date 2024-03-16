//LICENCE

#ifndef ROS_TO_IMC_FOLLOWREFERENCE_H
#define ROS_TO_IMC_FOLLOWREFERENCE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/FollowReference.h"
#include <IMC/Spec/FollowReference.hpp>
#include <IMC/Spec/PlanControl.hpp>
#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/PlanSpecification.hpp>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::FollowReference& ros_msg, IMC::PlanControl& imc_msg);

} // namespace ros_to_imc
#endif 
