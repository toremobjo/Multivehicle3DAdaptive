// LICENCE
#include <imc_ros_bridge/ros_to_imc/Sms.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::Sms& ros_msg, IMC::Sms& imc_msg)
{
    imc_msg.number = ros_msg.number.data;
    imc_msg.timeout = ros_msg.timeout.data;
    imc_msg.contents = ros_msg.contents.data;

    return true;
}

} // namespace imc_to_ros
