
#include <imc_ros_bridge/imc_to_ros/IridiumTxStatus.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::IridiumTxStatus& imc_msg, imc_ros_interface::IridiumTxStatus& ros_msg)
{
    ros_msg.req_id.data = imc_msg.req_id;
    ros_msg.status.data = imc_msg.status;
    ros_msg.text.data = imc_msg.text;
    return true;
}

} // namespace imc_to_ros
