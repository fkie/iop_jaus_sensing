/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#ifndef DIGITALVIDEO_RECEIVEFSM_H
#define DIGITALVIDEO_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_DigitalVideo/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_DigitalVideo/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h"
#include "urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClientService.h"


#include "DigitalVideo_ReceiveFSM_sm.h"
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <fkie_iop_digital_resource_discovery/DigitalResourceEndpoint.h>

namespace urn_jaus_jss_environmentSensing_DigitalVideo
{

class DllExport DigitalVideo_ReceiveFSM : public JTS::StateMachine
{
public:
	DigitalVideo_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~DigitalVideo_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void modifyDigitalVideoSensorStreamAction(ControlDigitalVideoSensorStream msg);
	virtual void sendConfirmSensorConfigurationAction(SetDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportDigitalVideoSensorCapabilitiesAction(QueryDigitalVideoSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportDigitalVideoSensorConfigurationAction(QueryDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void updateDigitalVideoSensorConfigurationAction(SetDigitalVideoSensorConfiguration msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);



	DigitalVideo_ReceiveFSMContext *context;

protected:

	class VideoEndpoint {
	public:
		VideoEndpoint(std::shared_ptr<iop::Component> cmp, int id, int type, std::string url, DigitalVideo_ReceiveFSM& parent);
		~VideoEndpoint();
		digital_resource_endpoint::DigitalResourceEndpoint endpoint;
	private:
		std::shared_ptr<iop::Component> cmp;
		DigitalVideo_ReceiveFSM* parent;
		int ep_id;
		std::string ep_type;
		std::string ep_url;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rtsp_sub;
		void ros_video_rtsp_handler(const std_msgs::msg::String::SharedPtr msg);
	};

	/// References to parent FSMs
	urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;
	urn_jaus_jss_core_DiscoveryClient::DiscoveryClientService* p_discovery_client_service;
	urn_jaus_jss_iop_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService *p_ds_discovery_client_service;
	JausAddress p_digital_resource_discovery_addr;
	std::map<int, std::shared_ptr<VideoEndpoint> > p_endpoints;
	rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr p_pub_ressource_id;

	void discovered(const std::string &service_uri, JausAddress &iop_address);
	void pRegisterVideo(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
	void pUnregisterVideo(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
};

}

#endif // DIGITALVIDEO_RECEIVEFSM_H
