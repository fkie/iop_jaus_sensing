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


#include "urn_jaus_jss_environmentSensing_DigitalVideo/DigitalVideo_ReceiveFSM.h"




using namespace JTS;
using namespace urn_jaus_jss_core_DiscoveryClient;
using namespace urn_jaus_jss_iop_DigitalResourceDiscoveryClient;
using namespace digital_resource_endpoint;

namespace urn_jaus_jss_environmentSensing_DigitalVideo
{

static unsigned short sensor_id = 1;

DigitalVideo_ReceiveFSM::DigitalVideo_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new DigitalVideo_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pVisualSensor_ReceiveFSM = pVisualSensor_ReceiveFSM;
	p_discovery_client_service = NULL;
	p_ds_discovery_client_service = NULL;

}



DigitalVideo_ReceiveFSM::~DigitalVideo_ReceiveFSM()
{
	delete context;
}

void DigitalVideo_ReceiveFSM::setupNotifications()
{
	pVisualSensor_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_DigitalVideo_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	pVisualSensor_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_DigitalVideo_ReceiveFSM_Receiving_Ready_Controlled", "VisualSensor_ReceiveFSM");
	pVisualSensor_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DigitalVideo_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	pVisualSensor_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DigitalVideo_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_NotControlled", "DigitalVideo_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_Controlled", "DigitalVideo_ReceiveFSM");
	registerNotification("Receiving_Ready", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready", "DigitalVideo_ReceiveFSM");
	registerNotification("Receiving", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving", "DigitalVideo_ReceiveFSM");

	p_pub_ressource_id = n.advertise<std_msgs::UInt16>("dv_resource_id", 10, true);
	p_discovery_client_service = dynamic_cast<DiscoveryClientService*>(iop::Component::get_instance().get_service("DiscoveryClient"));
	if (p_discovery_client_service == NULL)
		throw std::runtime_error("DiscoveryClientService not found, need to discover urn:jaus:jss:iop:DigitalResourceDiscovery");
	p_ds_discovery_client_service = dynamic_cast<DigitalResourceDiscoveryClientService*>(iop::Component::get_instance().get_service("DigitalResourceDiscoveryClient"));
	if (p_ds_discovery_client_service == NULL)
		throw std::runtime_error("DigitalResourceDiscoveryClientService not found, needed by DigitalVideo");
	ros::NodeHandle pnh("~");
	std::string rtsp_topic = "";
	std::string mjpeg_uri = "";
	XmlRpc::XmlRpcValue video_endpoints;
	pnh.getParam("video_endpoints", video_endpoints);
	if (!video_endpoints.valid()) {
		ROS_ERROR("wrong '~video_endpoints' format, expected list of: ID/TYPE: URL.\n  ID: ressource id {0..254}\n  TYPE: server type {rtsp_topic, rtsp, http, https, ftp, sftp, ftp_ssh, scp, mpeg2ts}");
		ROS_BREAK();
	}
	// parse the parameter into a map
	std::map<std::string, std::string > endpoints_list;
	for(int i = 0; i < video_endpoints.size(); i++) {
		if (video_endpoints[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = video_endpoints[i].begin(); iterator != video_endpoints[i].end(); iterator++) {
				std::string vi_type = iterator->first;
				std::string vi_uri = iterator->second;
				endpoints_list[vi_type] = vi_uri;
			}
		} else {
			ROS_ERROR("wrong entry of '~video_endpoints', expected: ID/TYPE/URL.\n  ID: int value 0-254\n  TYPE: {rtsp_topic, rtsp, http, https, ftp, sftp, ftp_ssh, scp, mpeg2ts}");
		}
	}

	for(std::map<std::string, std::string >::const_iterator it = endpoints_list.begin(); it != endpoints_list.end(); ++it) {
		std::string delimiter = "/";
		size_t pos = 0;
		std::string ep_type = it->first;
		std::string ep_id_str;
		if ((pos = ep_type.find(delimiter)) != std::string::npos) {
			ep_id_str = ep_type.substr(0, pos);
			ep_type.erase(0, pos + delimiter.length());
		} else {
			ROS_ERROR("wrong type definition format, expected : ID/TYPE [ID: ressource id {0..254} TYPE: server type {rtsp_topic, rtsp, http, https, ftp, sftp, ftp_ssh, scp, mpeg2ts}], found: %s", ep_type.c_str());
		}
		int ep_id = std::atoi(ep_id_str.c_str());
		ROS_INFO_STREAM_NAMED("DigitalVideo", "Found digital endpoint: " << (std::string)(it->first) << " ==> " << it->second);
		std::string ep_addr = (std::string)(it->second);
		ROS_INFO_STREAM_NAMED("DigitalVideo", "  type: " << ep_type << " ==> " << ep_addr);
		DigitalResourceEndpoint endpoint;
		if (ep_type.compare("rtsp_topic") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_RTSP;
			ROS_INFO_NAMED("DigitalVideo", "    get RTPS from topic: %s", ep_addr.c_str());
			p_topics_map[ep_addr] = ep_id;
			p_endpoints[ep_id] = endpoint;
			p_subscriber[ep_addr] = n.subscribe(ep_addr, 1, &DigitalVideo_ReceiveFSM::ros_video_rtsp_handler, this);
		} else if (ep_type.compare("rtsp") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_RTSP;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    RTPS: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("http") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_HTTP;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    HTTP: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("https") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_HTTPS;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    HTTPS: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("ftp") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_FTP;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    FTP: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("sftp") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_SFTP;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    SFTP: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("ftp_ssh") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_FTP_SSH;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    SFTP: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("scp") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_SCP;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    SCP: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		} else if (ep_type.compare("mpeg2ts") == 0) {
			endpoint.resource_id = ep_id;
			endpoint.server_type = digital_resource_endpoint::SERVER_TYPE_MPEG2TS;
			endpoint.server_url = ep_addr;
			ROS_INFO_NAMED("DigitalVideo", "    MPEG2TS: %s", ep_addr.c_str());
			p_endpoints[ep_id] = endpoint;
		}
	}
	p_discovery_client_service->pDiscoveryClient_ReceiveFSM->discover("urn:jaus:jss:iop:DigitalResourceDiscovery", &DigitalVideo_ReceiveFSM::discovered, this, 1, 0, jausRouter->getJausAddress()->getSubsystemID());

}

void DigitalVideo_ReceiveFSM::modifyDigitalVideoSensorStreamAction(ControlDigitalVideoSensorStream msg)
{
	/// Insert User Code HERE
	unsigned short id = msg.getBody()->getControlDigitalVideoSensorStreamRec()->getSensorID();
	ROS_DEBUG_NAMED("DigitalVideo", "modifyDigitalVideoSensorStreamAction resource %d", id);
	if (msg.getBody()->getControlDigitalVideoSensorStreamRec()->getStreamState() == 2) {
		id = 65535;
	} else if (msg.getBody()->getControlDigitalVideoSensorStreamRec()->getStreamState() == 0) {
	}
	std_msgs::UInt16 cmd;
	cmd.data = id;
	p_pub_ressource_id.publish(cmd);
}

void DigitalVideo_ReceiveFSM::sendConfirmSensorConfigurationAction(SetDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("DigitalVideo", "sendConfirmSensorConfigurationAction to %d.%d.%d", subsystem_id, node_id, component_id);
	unsigned char request_id = msg.getBody()->getDigitalVideoSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	ConfirmSensorConfiguration response;
	response.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->setRequestID(request_id);
	sendJausMessage(response,sender);
}

void DigitalVideo_ReceiveFSM::sendReportDigitalVideoSensorCapabilitiesAction(QueryDigitalVideoSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("DigitalVideo", "sendReportDigitalVideoSensorCapabilitiesAction to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportDigitalVideoSensorCapabilities response;
	ReportDigitalVideoSensorCapabilities::Body::DigitalVideoSensorList::DigitalVideoSensorCapabilitiesRec entry;
	entry.setSensorID(sensor_id);
	response.getBody()->getDigitalVideoSensorList()->addElement(entry);
	sendJausMessage(response, sender);
}

void DigitalVideo_ReceiveFSM::sendReportDigitalVideoSensorConfigurationAction(QueryDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("DigitalVideo", "sendReportDigitalVideoSensorConfigurationAction to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportDigitalVideoSensorConfiguration response;
	ReportDigitalVideoSensorConfiguration::Body::DigitalVideoSensorConfigurationList::DigitalVideoSensorConfigurationRec entry;
	entry.setSensorID(sensor_id);
	response.getBody()->getDigitalVideoSensorConfigurationList()->addElement(entry);
	sendJausMessage(response, sender);
}

void DigitalVideo_ReceiveFSM::updateDigitalVideoSensorConfigurationAction(SetDigitalVideoSensorConfiguration msg)
{
	/// Insert User Code HERE
	ROS_WARN_NAMED("DigitalVideo", "updateDigitalVideoSensorConfigurationAction not implemented");
}



bool DigitalVideo_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}


void DigitalVideo_ReceiveFSM::discovered(const std::string &service_uri, JausAddress &iop_address)
{
	ROS_INFO_NAMED("DigitalVideo", "discovered %s @ %d.%d.%d", service_uri.c_str(), iop_address.getSubsystemID(), iop_address.getNodeID(), iop_address.getComponentID());
	if (p_digital_resource_discovery_addr.get() != iop_address.get()) {
		p_digital_resource_discovery_addr = iop_address;
		for (std::map<int, DigitalResourceEndpoint>::iterator it = p_endpoints.begin(); it != p_endpoints.end(); ++it) {
			pRegisterVideo(it->second);
		}
	}
}

void DigitalVideo_ReceiveFSM::ros_video_rtsp_handler(const ros::MessageEvent<const std_msgs::String >& event)
{
	/*
	* Store the URL of the rtsp stream. This will be returned by discovering.
	*/
	const std_msgs::String& msg = *event.getMessage();
	std::string topic_name = event.getConnectionHeader()["topic"];
	std::map<std::string, int>::iterator it = p_topics_map.find(topic_name);
	if (it != p_topics_map.end()) {
		p_endpoints[p_topics_map[topic_name]].server_url = msg.data;
		ROS_INFO_NAMED("DigitalVideo", "received RTSP URL: %s\n", msg.data.c_str());
		pRegisterVideo(p_endpoints[p_topics_map[topic_name]]);
	}
}

void DigitalVideo_ReceiveFSM::pRegisterVideo(DigitalResourceEndpoint endpoint)
{
	if (!endpoint.server_url.empty() && p_digital_resource_discovery_addr.get() != 0) {
		ROS_INFO_NAMED("DigitalVideo", "register video %s on subsystem %d", endpoint.server_url.c_str(), p_digital_resource_discovery_addr.getSubsystemID());
		endpoint.iop_id = *(jausRouter->getJausAddress());
		p_ds_discovery_client_service->pDigitalResourceDiscoveryClient_ReceiveFSM->registerEndpoint(endpoint, p_digital_resource_discovery_addr);
	}
}

void DigitalVideo_ReceiveFSM::pUnregisterVideo(DigitalResourceEndpoint endpoint)
{
	if (!endpoint.server_url.empty() && p_digital_resource_discovery_addr.get() != 0) {
		ROS_INFO_NAMED("DigitalVideo", "unregister video %s on subsystem %d", endpoint.server_url.c_str(), p_digital_resource_discovery_addr.getSubsystemID());
		endpoint.iop_id = *(jausRouter->getJausAddress());
		p_ds_discovery_client_service->pDigitalResourceDiscoveryClient_ReceiveFSM->unregisterEndpoint(endpoint, p_digital_resource_discovery_addr);
	}
}


};
