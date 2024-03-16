// LICENCE
#ifndef ROS_TO_IMC_IRIDIUMMSGTX
#define ROS_TO_IMC_IRIDIUMMSGTX

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/IridiumMsgTx.h"
#include <IMC/Spec/IridiumMsgTx.hpp>


namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::IridiumMsgTx& ros_msg, IMC::IridiumMsgTx& imc_msg);

} // namespace ros_to_imc
#endif
