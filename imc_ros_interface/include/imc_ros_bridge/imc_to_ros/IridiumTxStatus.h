// LICENCE

#ifndef IMC_TO_ROS_IRIDIUMTXSTATUS
#define IMC_TO_ROS_IRIDIUMTXSTATUS

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_interface/IridiumTxStatus.h>
#include <IMC/Spec/IridiumTxStatus.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::IridiumTxStatus& imc_msg, imc_ros_interface::IridiumTxStatus& ros_msg);

}

#endif
