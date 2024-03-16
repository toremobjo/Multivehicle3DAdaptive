// LICENCE
#include <imc_ros_bridge/ros_to_imc/Reference.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::Reference& ros_msg, IMC::Reference& imc_msg)
{
    imc_msg.flags = ros_msg.flags.data;

    IMC::DesiredSpeed speed;
    speed.value = ros_msg.speed.value.data;
    speed.speed_units = ros_msg.speed.speed_units.data;
    imc_msg.speed.set(speed);

    IMC::DesiredZ desiredz;
    desiredz.value = ros_msg.z.value.data;
    desiredz.z_units = ros_msg.z.z_units.data;
    imc_msg.z.set(desiredz);

    imc_msg.lat = ros_msg.lat.data;
    imc_msg.lon = ros_msg.lon.data;
    imc_msg.radius = ros_msg.radius.data;
    imc_msg.setSource(ros_msg.source.data);


    return true;
}

} // namespace imc_to_ros
