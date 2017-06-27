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

typedef JTS::Receive Receive;
typedef JTS::Send Send;

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h"
#include "urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClientService.h"


#include "DigitalVideo_ReceiveFSM_sm.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iop_digital_resource_discovery_fkie/DigitalResourceEndpoint.h>
#include <iop_component_fkie/iop_component.h>

namespace urn_jaus_jss_environmentSensing_DigitalVideo
{

class DllExport DigitalVideo_ReceiveFSM : public JTS::StateMachine
{
public:
	DigitalVideo_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM);
	virtual ~DigitalVideo_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

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

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM;

	urn_jaus_jss_core_DiscoveryClient::DiscoveryClientService* p_discovery_client_service;
	urn_jaus_jss_iop_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService *p_ds_discovery_client_service;
	JausAddress p_digital_resource_discovery_addr;
	std::map<int, digital_resource_endpoint::DigitalResourceEndpoint> p_endpoints;
	std::map<std::string, int> p_topics_map;
	std::map<std::string, ros::Subscriber> p_subscriber;

	void discovered(const std::string &service_uri, JausAddress &iop_address);
	void pRegisterVideo(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
	void pUnregisterVideo(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
	void ros_video_rtsp_handler(const ros::MessageEvent<const std_msgs::String >& event);
};

};

#endif // DIGITALVIDEO_RECEIVEFSM_H
