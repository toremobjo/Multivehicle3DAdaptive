// LICENCE
#include <imc_ros_bridge/ros_to_imc/ReportControl.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::ReportControl& ros_msg, IMC::ReportControl& imc_msg)
{
    imc_msg.op = ros_msg.op.data;
    imc_msg.comm_interface = ros_msg.comm_interface.data;
    imc_msg.sys_dst = ros_msg.sys_dst.data;
    imc_msg.period = ros_msg.period.data;

    return true;
}

} // namespace imc_to_ros
