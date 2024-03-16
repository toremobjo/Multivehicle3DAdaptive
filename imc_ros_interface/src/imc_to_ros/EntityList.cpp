// LICENCE

#include <imc_ros_bridge/imc_to_ros/EntityList.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::EntityList& imc_msg, imc_ros_interface::EntityList& ros_msg)
{
    // Header
    ros_msg.header.mgid.data = imc_msg.getId();
    ros_msg.header.size.data  = imc_msg.getVariableSerializationSize();
    ros_msg.header.timestamp.data = imc_msg.getTimeStamp();
    ros_msg.header.src.data = imc_msg.getSource();
    ros_msg.header.src_ent.data = imc_msg.getSourceEntity();
    // Message
    ros_msg.op.data = imc_msg.op;
    ros_msg.list.data = imc_msg.list;

    return true;
}

} // namespace imc_to_ros
