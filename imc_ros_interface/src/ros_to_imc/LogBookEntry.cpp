// LICENCE
#include <imc_ros_bridge/ros_to_imc/LogBookEntry.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::LogBookEntry& ros_msg, IMC::LogBookEntry& imc_msg)
{
    imc_msg.type = ros_msg.type.data;
    imc_msg.htime = ros_msg.htime.data;
    imc_msg.context = ros_msg.context.data;
    imc_msg.text = ros_msg.text.data;

    return true;
}

} // namespace imc_to_ros
