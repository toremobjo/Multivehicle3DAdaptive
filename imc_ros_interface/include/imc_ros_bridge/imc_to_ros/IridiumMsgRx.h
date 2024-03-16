// LICENCE

#ifndef IMC_TO_ROS_IRIDIUMMSGRX
#define IMC_TO_ROS_IRIDIUMMSGRX

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/IridiumMsgRx.h>
#include <IMC/Spec/IridiumMsgRx.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::IridiumMsgRx& imc_msg, imc_ros_interface::IridiumMsgRx& ros_msg);

}

#endif
