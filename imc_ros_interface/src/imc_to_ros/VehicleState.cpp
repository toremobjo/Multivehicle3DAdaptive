// LICENCE

#include <imc_ros_bridge/imc_to_ros/VehicleState.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::VehicleState& imc_msg, imc_ros_interface::VehicleState& ros_msg)
{
    // Header
    ros_msg.header.mgid.data = imc_msg.getId();
    ros_msg.header.size.data  = imc_msg.getVariableSerializationSize();
    ros_msg.header.timestamp.data = imc_msg.getTimeStamp();
    ros_msg.header.src.data = imc_msg.getSource();
    ros_msg.header.src_ent.data = imc_msg.getSourceEntity();
    // Message
    ros_msg.op_mode.data = imc_msg.op_mode;
    ros_msg.error_count.data = imc_msg.error_count;
    ros_msg.error_ents.data = imc_msg.error_ents;
    ros_msg.maneuver_type.data = imc_msg.maneuver_type;
    ros_msg.maneuver_stime.data = imc_msg.maneuver_stime;
    ros_msg.maneuver_eta.data = imc_msg.maneuver_eta;
    ros_msg.control_loops.data = imc_msg.control_loops;
    ros_msg.flags.data = imc_msg.flags;
    ros_msg.last_error.data = imc_msg.last_error;
    ros_msg.last_error_time.data = imc_msg.last_error_time;

    return true;
}

} // namespace imc_to_ros
