
#include <imc_ros_bridge/imc_to_ros/IridiumMsgRx.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::IridiumMsgRx& imc_msg, imc_ros_interface::IridiumMsgRx& ros_msg)
{
    ros_msg.origin.data = imc_msg.origin;
    ros_msg.htime.data  = imc_msg.htime;
    ros_msg.lat.data    = imc_msg.lat;
    ros_msg.lon.data    = imc_msg.lon;

    std::vector<char> array = imc_msg.data;
    std::vector<signed char> vec(array.begin(), array.end());
    ros_msg.data.data = vec;

    return true;
}

} // namespace imc_to_ros
