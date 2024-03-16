//Licence
#include <imc_ros_bridge/ros_to_imc/FollowReference.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::FollowReference& ros_msg, IMC::PlanControl& imc_msg)
{
    std::string plan_id_ = "adaframe_ntnu";

    imc_msg.plan_id = plan_id_;
    imc_msg.op = IMC::PlanControl::PC_START;
    imc_msg.type = IMC::PlanControl::PC_REQUEST;
    imc_msg.request_id = 1000;

    IMC::FollowReference man;
    man.control_src = ros_msg.control_src.data;
    man.control_ent = ros_msg.control_ent.data;
    man.loiter_radius = ros_msg.loiter_radius.data;
    man.timeout = ros_msg.timeout.data;
    man.altitude_interval = ros_msg.altitude_interval.data;

    IMC::PlanManeuver pm;
    pm.maneuver_id = "1";
    pm.data.set(man);

    IMC::PlanSpecification ps;
    ps.plan_id = imc_msg.plan_id;
    ps.start_man_id = pm.maneuver_id;
    ps.maneuvers.push_back(pm);

    imc_msg.arg.set(ps);
    return true;
}

} // namespace imc_to_ros
