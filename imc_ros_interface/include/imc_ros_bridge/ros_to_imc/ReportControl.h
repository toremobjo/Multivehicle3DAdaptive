// LICENCE
#ifndef ROS_TO_IMC_REPORTCONTROL
#define ROS_TO_IMC_REPORTCONTROL

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_interface/ReportControl.h"
#include <IMC/Spec/ReportControl.hpp>


namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::ReportControl& ros_msg, IMC::ReportControl& imc_msg);

} // namespace ros_to_imc
#endif
