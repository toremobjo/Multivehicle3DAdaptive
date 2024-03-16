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

#include <list>
#include <md5.h>

#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_interface/PlanSpecification.h>
#include <imc_ros_interface/PlanManeuver.h>

#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/Message.hpp>
#include <IMC/Base/MessageList.hpp>
// #include <IMC/Base/Packet.hpp> this has the IMC::serialize function in it.

#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/Goto.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanDB& imc_msg, imc_ros_interface::PlanDB& ros_msg)
{
	ros_msg.type = imc_msg.type;
	ros_msg.op = imc_msg.op;
	ros_msg.request_id = imc_msg.request_id;
	ros_msg.plan_id = imc_msg.plan_id;

	//IMC::InlineMessage<IMC::Message> arg = imc_msg.arg;
	//if(arg.isNull()){
	return true;


}


} // namespace imc_to_ros
