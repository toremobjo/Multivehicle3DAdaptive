/* Copyright 2019 The SMaRC project (https://smarc.se/)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/imc_ros_bridge_server.h>

#include <imc_ros_bridge/ros_to_imc/Goto.h>
#include <imc_ros_bridge/ros_to_imc/RemoteState.h>
#include <imc_ros_bridge/ros_to_imc/SonarData.h>
#include <imc_ros_bridge/ros_to_imc/DesiredHeading.h>
#include <imc_ros_bridge/ros_to_imc/DesiredHeadingRate.h>
#include <imc_ros_bridge/ros_to_imc/DesiredPitch.h>
#include <imc_ros_bridge/ros_to_imc/DesiredRoll.h>
#include <imc_ros_bridge/ros_to_imc/PlanDB.h>
#include <imc_ros_bridge/ros_to_imc/EntityList.h>
#include <imc_ros_bridge/ros_to_imc/FollowReference.h>
#include <imc_ros_bridge/ros_to_imc/Reference.h>
#include <imc_ros_bridge/ros_to_imc/LogBookEntry.h>
#include <imc_ros_bridge/ros_to_imc/Sms.h>
#include <imc_ros_bridge/ros_to_imc/IridiumMsgTx.h>
#include <imc_ros_bridge/ros_to_imc/ReportControl.h>

#include <imc_ros_bridge/imc_to_ros/EntityState.h>
#include <imc_ros_bridge/imc_to_ros/Goto.h>
#include <imc_ros_bridge/imc_to_ros/Heartbeat.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_bridge/imc_to_ros/PlanControl.h>
#include <imc_ros_bridge/imc_to_ros/Temperature.h>
#include <imc_ros_bridge/imc_to_ros/Depth.h>
#include <imc_ros_bridge/imc_to_ros/VehicleState.h>
#include <imc_ros_bridge/imc_to_ros/EstimatedState.h>
#include <imc_ros_bridge/imc_to_ros/EntityList.h>
#include <imc_ros_bridge/imc_to_ros/VehicleMedium.h>
#include <imc_ros_bridge/imc_to_ros/PlanControlState.h>
#include <imc_ros_bridge/imc_to_ros/FollowRefState.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/Salinity.h>
#include <imc_ros_bridge/imc_to_ros/Chlorophyll.h>
#include <imc_ros_bridge/imc_to_ros/IridiumTxStatus.h>
#include <imc_ros_bridge/imc_to_ros/IridiumMsgRx.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imc_bridge");
    ros::NodeHandle ros_node;

    std::string bridge_tcp_addr;
    std::string bridge_tcp_port;

    ros::param::param<std::string>("~bridge_addr", bridge_tcp_addr, "127.0.0.1");
    ros::param::param<std::string>("~bridge_port", bridge_tcp_port, "32603");

    IMCHandle imc_handle(bridge_tcp_addr, bridge_tcp_port);

    // ######### ROS TO IMC ###########################
    // 5
    ros_to_imc::BridgeServer<imc_ros_interface::EntityList, IMC::EntityList> to_imc_EntityList_server(ros_node, imc_handle, "/IMC/In/EntityList");
    // 103
    ros_to_imc::BridgeServer<imc_ros_interface::LogBookEntry, IMC::LogBookEntry> to_imc_LogBookEntry_server(ros_node, imc_handle, "/IMC/In/LogBookEntry");
    // 156
    ros_to_imc::BridgeServer<imc_ros_interface::Sms, IMC::Sms> to_imc_Sms_server(ros_node, imc_handle, "/IMC/In/Sms");
    // 171
    ros_to_imc::BridgeServer<imc_ros_interface::IridiumMsgTx, IMC::IridiumMsgTx> to_imc_IridiumMsgTx_server(ros_node, imc_handle, "/IMC/In/IridiumMsgTx");
    // 478
    ros_to_imc::BridgeServer<imc_ros_interface::FollowReference, IMC::PlanControl> to_imc_FollowReference_server(ros_node, imc_handle, "/IMC/In/FollowReference");
    // 479
    ros_to_imc::BridgeServer<imc_ros_interface::Reference, IMC::Reference> to_imc_Reference_server(ros_node, imc_handle, "/IMC/In/Reference");
    // 513
    ros_to_imc::BridgeServer<imc_ros_interface::ReportControl, IMC::ReportControl> to_imc_ReportControl_server(ros_node,imc_handle,"/IMC/In/ReportControl");

    // ######### IMC TO ROS ########################
    // 1
    imc_to_ros::BridgeServer<IMC::EntityState, imc_ros_interface::EntityState> EntityState_server(imc_handle, ros_node, "/IMC/Out/EntityState");
    // 5
    imc_to_ros::BridgeServer<IMC::EntityList, imc_ros_interface::EntityList> EntityList_server(imc_handle, ros_node, "/IMC/Out/EntityList");
    // 150
    imc_to_ros::BridgeServer<IMC::Heartbeat, imc_ros_interface::Heartbeat> Heartbeat_server(imc_handle, ros_node, "/IMC/Out/Heartbeat");
    // 170
    imc_to_ros::BridgeServer<IMC::IridiumMsgRx, imc_ros_interface::IridiumMsgRx> IridiumMsgRx_server(imc_handle, ros_node, "/IMC/Out/IridiumMsgRx");
    // 172
    imc_to_ros::BridgeServer<IMC::IridiumTxStatus, imc_ros_interface::IridiumTxStatus> IridiumTxStatus_server(imc_handle, ros_node, "/IMC/Out/IridiumTxStatus");
    // 263
    imc_to_ros::BridgeServer<IMC::Temperature, imc_ros_interface::Temperature> Temperature_server(imc_handle, ros_node, "/IMC/Out/Temperature");
    // 265
    imc_to_ros::BridgeServer<IMC::Depth, imc_ros_interface::Depth> Depth_server(imc_handle, ros_node, "/IMC/Out/Depth");
    // 270
    imc_to_ros::BridgeServer<IMC::Salinity, imc_ros_interface::Salinity> Salinity_server(imc_handle, ros_node, "/IMC/Out/Salinity");
    // 289
    imc_to_ros::BridgeServer<IMC::Chlorophyll, imc_ros_interface::Chlorophyll> Chlorophyll_server(imc_handle, ros_node, "/IMC/Out/Chlorophyll");
    // 350
    imc_to_ros::BridgeServer<IMC::EstimatedState, imc_ros_interface::EstimatedState> EstimatedState_server(imc_handle, ros_node, "/IMC/Out/EstimatedState");
    // 480
    imc_to_ros::BridgeServer<IMC::FollowRefState, imc_ros_interface::FollowRefState> FollowRefState_server(imc_handle, ros_node, "/IMC/Out/FollowRefState");
    // 500
    imc_to_ros::BridgeServer<IMC::VehicleState, imc_ros_interface::VehicleState> VehicleState_server(imc_handle, ros_node, "/IMC/Out/VehicleState");
    // 508
    imc_to_ros::BridgeServer<IMC::VehicleMedium, imc_ros_interface::VehicleMedium> VehicleMedium_server(imc_handle, ros_node, "/IMC/Out/VehicleMedium");
    // 559
    imc_to_ros::BridgeServer<IMC::PlanControl, imc_ros_interface::PlanControl> PlanControl_server(imc_handle, ros_node, "/IMC/Out/PlanControl");
    // 560
    imc_to_ros::BridgeServer<IMC::PlanControlState, imc_ros_interface::PlanControlState> PlanControlState_server(imc_handle, ros_node, "/IMC/Out/PlanControlState");

    ros::spin();
    return 0;
}
