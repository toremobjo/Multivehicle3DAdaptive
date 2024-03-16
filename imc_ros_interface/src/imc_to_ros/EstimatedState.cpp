// LICENCE

#include <imc_ros_bridge/imc_to_ros/EstimatedState.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::EstimatedState& imc_msg, imc_ros_interface::EstimatedState& ros_msg)
{
    // Header
    ros_msg.header.mgid.data = imc_msg.getId();
    ros_msg.header.size.data  = imc_msg.getVariableSerializationSize();
    ros_msg.header.timestamp.data = imc_msg.getTimeStamp();
    ros_msg.header.src.data = imc_msg.getSource();
    ros_msg.header.src_ent.data = imc_msg.getSourceEntity();
    // Message
    ros_msg.lat.data = imc_msg.lat;
    ros_msg.lon.data = imc_msg.lon;
    ros_msg.height.data = imc_msg.height;
    ros_msg.x.data = imc_msg.x;
    ros_msg.y.data = imc_msg.y;
    ros_msg.z.data = imc_msg.z;
    ros_msg.phi.data = imc_msg.phi;
    ros_msg.theta.data = imc_msg.theta;
    ros_msg.psi.data = imc_msg.psi;
    ros_msg.u.data = imc_msg.u;
    ros_msg.v.data = imc_msg.v;
    ros_msg.w.data = imc_msg.w;
    ros_msg.vx.data = imc_msg.vx;
    ros_msg.vy.data = imc_msg.vy;
    ros_msg.vz.data = imc_msg.vz;
    ros_msg.p.data = imc_msg.p;
    ros_msg.q.data = imc_msg.q;
    ros_msg.r.data = imc_msg.r;
    ros_msg.depth.data = imc_msg.depth;
    ros_msg.alt.data = imc_msg.alt;

    return true;
}

} // namespace imc_to_ros
