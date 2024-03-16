// LICENCE


#include <imc_ros_bridge/imc_to_ros/FollowRefState.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::FollowRefState& imc_msg, imc_ros_interface::FollowRefState& ros_msg)
{
    // Header
    ros_msg.header.mgid.data = imc_msg.getId();
    ros_msg.header.size.data  = imc_msg.getVariableSerializationSize();
    ros_msg.header.timestamp.data = imc_msg.getTimeStamp();
    ros_msg.header.src.data = imc_msg.getSource();
    ros_msg.header.src_ent.data = imc_msg.getSourceEntity();
    // Message
    ros_msg.control_src.data = imc_msg.control_src;
    ros_msg.control_ent.data = imc_msg.control_ent;

/*
    IMC::Reference ref;
    ref=imc_msg.reference;
    ros_msg.reference.flags.data = ref.flags;

    IMC::DesiredSpeed dspeed = ref.speed;
    ros_msg.reference.speed.value.data = dspeed.value;
    ros_msg.reference.speed.speed_units.data = dspeed.speed_units;

    IMC::DesiredZ dz = ref.z;
    ros_msg.reference.z.value.data = dz.value;
    ros_msg.reference.z.z_units.data = dz.z_units;


    ros_msg.reference.lat.data = ref.lat;
    ros_msg.reference.lon.data = ref.lon;
    ros_msg.reference.radius.data = ref.radius;
*/

    ros_msg.state.data = imc_msg.state;
    ros_msg.proximity.data = imc_msg.proximity;

    return true;
}

} // namespace imc_to_ros
