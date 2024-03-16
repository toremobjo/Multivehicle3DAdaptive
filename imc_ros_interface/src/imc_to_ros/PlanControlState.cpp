// LICENCE

#include <imc_ros_bridge/imc_to_ros/PlanControlState.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanControlState& imc_msg, imc_ros_interface::PlanControlState& ros_msg)
{
    // Header
    ros_msg.header.mgid.data = imc_msg.getId();
    ros_msg.header.size.data  = imc_msg.getVariableSerializationSize();
    ros_msg.header.timestamp.data = imc_msg.getTimeStamp();
    ros_msg.header.src.data = imc_msg.getSource();
    ros_msg.header.src_ent.data = imc_msg.getSourceEntity();
    // Message
    ros_msg.state.data = imc_msg.state;
    ros_msg.plan_id.data = imc_msg.plan_id;
    ros_msg.plan_eta.data = imc_msg.plan_eta;
    ros_msg.plan_progress.data = imc_msg.plan_progress;
    ros_msg.man_id.data = imc_msg.man_id;
    ros_msg.man_eta.data = imc_msg.man_eta;
    ros_msg.last_outcome.data = imc_msg.last_outcome;

    return true;
}

} // namespace imc_to_ros
