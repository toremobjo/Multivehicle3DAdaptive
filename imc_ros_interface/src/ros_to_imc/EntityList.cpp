// LICENCE
#include <imc_ros_bridge/ros_to_imc/EntityList.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::EntityList& ros_msg, IMC::EntityList& imc_msg)
{
    imc_msg.op = ros_msg.op.data;
    imc_msg.list = ros_msg.list.data;
    return true;
}

} // namespace imc_to_ros
