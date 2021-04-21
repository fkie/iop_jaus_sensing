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
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/string.hpp>




using namespace JTS;
using namespace urn_jaus_jss_core_DiscoveryClient;
using namespace urn_jaus_jss_iop_DigitalResourceDiscoveryClient;
using namespace digital_resource_endpoint;

namespace urn_jaus_jss_environmentSensing_DigitalVideo
{

static unsigned short sensor_id = 1;

DigitalVideo_ReceiveFSM::DigitalVideo_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("DigitalVideo"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new DigitalVideo_ReceiveFSMContext(*this);

	this->pVisualSensor_ReceiveFSM = pVisualSensor_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
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

}


void DigitalVideo_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "DigitalVideo");

	p_pub_ressource_id = cfg.create_publisher<std_msgs::msg::UInt16>("dv_resource_id", 10);
	p_discovery_client_service = dynamic_cast<DiscoveryClientService*>(cmp->get_service("DiscoveryClient"));
	if (p_discovery_client_service == NULL)
		throw std::runtime_error("DiscoveryClient service not found, need to discover urn:jaus:jss:iop:DigitalResourceDiscovery");
	p_ds_discovery_client_service = dynamic_cast<DigitalResourceDiscoveryClientService*>(cmp->get_service("DigitalResourceDiscoveryClient"));
	if (p_ds_discovery_client_service == NULL)
		throw std::runtime_error("DigitalResourceDiscoveryClientService not found, needed by DigitalVideo");
	std::vector<std::string> endpoints;
	cfg.declare_param<std::vector<std::string> >("endpoints", endpoints, false,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"list with video streams. Format: ID.TYPE_OF_SOURCE.URL. Example for RTSP stream: 0.rtsp.rtsp://URL",
		"Default: []");
	cfg.param_vector<std::vector<std::string> >("endpoints", endpoints, endpoints);
	for (unsigned int i = 0; i < endpoints.size(); i++) {
		auto entry = iop::split(iop::trim(endpoints[i]), '.', 3);
		if (entry.size() == 3) {
			int ep_id = std::atoi(entry[0].c_str());
			std::string str_type = entry[1];
			std::string ep_url = entry[2];
			RCLCPP_INFO(logger, "Found digital endpoint <id: %d, type: %s, url: %s>", ep_id, str_type, ep_url);
			int ep_type = 0;
			if (str_type.compare("rtsp_topic") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_RTSP;
				RCLCPP_INFO(logger, "    get RTPS from topic: %s", ep_url.c_str());
			} else if (str_type.compare("rtsp") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_RTSP;
				RCLCPP_INFO(logger, "    RTPS: %s", ep_url.c_str());
			} else if (str_type.compare("http") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_HTTP;
				RCLCPP_INFO(logger, "    HTTP: %s", ep_url.c_str());
			} else if (str_type.compare("https") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_HTTPS;
				RCLCPP_INFO(logger, "    HTTPS: %s", ep_url.c_str());
			} else if (str_type.compare("ftp") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_FTP;
				RCLCPP_INFO(logger, "    FTP: %s", ep_url.c_str());
			} else if (str_type.compare("sftp") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_SFTP;
				RCLCPP_INFO(logger, "    SFTP: %s", ep_url.c_str());
			} else if (str_type.compare("ftp_ssh") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_FTP_SSH;
				RCLCPP_INFO(logger, "    SFTP: %s", ep_url.c_str());
			} else if (str_type.compare("scp") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_SCP;
				RCLCPP_INFO(logger, "    SCP: %s", ep_url.c_str());
			} else if (str_type.compare("mpeg2ts") == 0) {
				ep_type = digital_resource_endpoint::SERVER_TYPE_MPEG2TS;
				RCLCPP_INFO(logger, "    MPEG2TS: %s", ep_url.c_str());
			}
			p_endpoints[ep_id] = std::make_shared<VideoEndpoint>(cmp, ep_id, ep_type, ep_url, *this);
		} else {
			RCLCPP_WARN(logger, "skipped endpoint entry '%s' because of invalid format, expected list of: ID.TYPE.URL.", endpoints[i]);
		}
	}
	p_discovery_client_service->pDiscoveryClient_ReceiveFSM->discover("urn:jaus:jss:iop:DigitalResourceDiscovery", &DigitalVideo_ReceiveFSM::discovered, this, 1, 255, jausRouter->getJausAddress()->getSubsystemID());

}

DigitalVideo_ReceiveFSM::VideoEndpoint::VideoEndpoint(std::shared_ptr<iop::Component> cmp, int id, int type, std::string url, DigitalVideo_ReceiveFSM& parent)
{
	endpoint.resource_id = id;
	endpoint.server_type = type;
	endpoint.server_url = url;
	this->cmp = cmp;
	this->parent = &parent;
	if (type == digital_resource_endpoint::SERVER_TYPE_RTSP) {
		iop::Config cfg(cmp, "VideoEndpoint");
		rtsp_sub = cfg.create_subscription<std_msgs::msg::String>(url, 1, std::bind(&DigitalVideo_ReceiveFSM::VideoEndpoint::ros_video_rtsp_handler, this, std::placeholders::_1));
	}
}

DigitalVideo_ReceiveFSM::VideoEndpoint::~VideoEndpoint()
{

}

void DigitalVideo_ReceiveFSM::VideoEndpoint::ros_video_rtsp_handler(const std_msgs::msg::String::SharedPtr msg)
{
	/*
	* Store the URL of the rtsp stream. This will be returned by discovering.
	*/
	endpoint.server_url = msg->data;
	RCLCPP_INFO(cmp->get_logger().get_child("VideoEndpoint"), "received RTSP URL: %s\n", msg->data.c_str());
	parent->pRegisterVideo(endpoint);
}

void DigitalVideo_ReceiveFSM::modifyDigitalVideoSensorStreamAction(ControlDigitalVideoSensorStream msg)
{
	/// Insert User Code HERE
	unsigned short id = msg.getBody()->getControlDigitalVideoSensorStreamRec()->getSensorID();
	RCLCPP_DEBUG(logger, "DigitalVideo", "modifyDigitalVideoSensorStreamAction resource %d", id);
	if (msg.getBody()->getControlDigitalVideoSensorStreamRec()->getStreamState() == 2) {
		id = 65535;
	} else if (msg.getBody()->getControlDigitalVideoSensorStreamRec()->getStreamState() == 0) {
	}
	auto cmd = std_msgs::msg::UInt16();
	cmd.data = id;
	p_pub_ressource_id->publish(cmd);
}

void DigitalVideo_ReceiveFSM::sendConfirmSensorConfigurationAction(SetDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "DigitalVideo", "sendConfirmSensorConfigurationAction to %s", sender.str().c_str());
	unsigned char request_id = msg.getBody()->getDigitalVideoSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	ConfirmSensorConfiguration response;
	response.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->setRequestID(request_id);
	sendJausMessage(response,sender);
}

void DigitalVideo_ReceiveFSM::sendReportDigitalVideoSensorCapabilitiesAction(QueryDigitalVideoSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "DigitalVideo", "sendReportDigitalVideoSensorCapabilitiesAction to %s", sender.str().c_str());
	ReportDigitalVideoSensorCapabilities response;
	std::map<int, std::shared_ptr<VideoEndpoint> >::iterator it;
	for (it = p_endpoints.begin(); it != p_endpoints.end(); it++) {
		ReportDigitalVideoSensorCapabilities::Body::DigitalVideoSensorList::DigitalVideoSensorCapabilitiesRec entry;
		entry.setSensorID(it->first);
		response.getBody()->getDigitalVideoSensorList()->addElement(entry);
	}
	sendJausMessage(response, sender);
}

void DigitalVideo_ReceiveFSM::sendReportDigitalVideoSensorConfigurationAction(QueryDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "DigitalVideo", "sendReportDigitalVideoSensorConfigurationAction to %s", sender.str().c_str());
	ReportDigitalVideoSensorConfiguration response;
	std::map<int, std::shared_ptr<VideoEndpoint> >::iterator it;
	for (it = p_endpoints.begin(); it != p_endpoints.end(); it++) {
		ReportDigitalVideoSensorConfiguration::Body::DigitalVideoSensorConfigurationList::DigitalVideoSensorConfigurationRec entry;
		entry.setSensorID(it->first);
		response.getBody()->getDigitalVideoSensorConfigurationList()->addElement(entry);
	}
	sendJausMessage(response, sender);
}

void DigitalVideo_ReceiveFSM::updateDigitalVideoSensorConfigurationAction(SetDigitalVideoSensorConfiguration msg)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "DigitalVideo", "updateDigitalVideoSensorConfigurationAction not implemented");
}



bool DigitalVideo_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}


void DigitalVideo_ReceiveFSM::discovered(const std::string &service_uri, JausAddress &iop_address)
{
	RCLCPP_INFO(logger, "DigitalVideo", "discovered %s @ %s", service_uri.c_str(), iop_address.str().c_str());
	p_digital_resource_discovery_addr = iop_address;
	for (std::map<int, std::shared_ptr<VideoEndpoint> >::iterator it = p_endpoints.begin(); it != p_endpoints.end(); ++it) {
		pRegisterVideo(it->second->endpoint);
	}
}

void DigitalVideo_ReceiveFSM::pRegisterVideo(DigitalResourceEndpoint endpoint)
{
	if (!endpoint.server_url.empty() && p_digital_resource_discovery_addr.get() != 0) {
		lock_type lock(p_mutex);
		RCLCPP_INFO(logger, "DigitalVideo", "register video %s on subsystem %d", endpoint.server_url.c_str(), p_digital_resource_discovery_addr.getSubsystemID());
		endpoint.iop_id = *(jausRouter->getJausAddress());
		p_ds_discovery_client_service->pDigitalResourceDiscoveryClient_ReceiveFSM->registerEndpoint(endpoint, p_digital_resource_discovery_addr);
	}
}

void DigitalVideo_ReceiveFSM::pUnregisterVideo(DigitalResourceEndpoint endpoint)
{
	if (!endpoint.server_url.empty() && p_digital_resource_discovery_addr.get() != 0) {
		lock_type lock(p_mutex);
		RCLCPP_INFO(logger, "DigitalVideo", "unregister video %s on subsystem %d", endpoint.server_url.c_str(), p_digital_resource_discovery_addr.getSubsystemID());
		endpoint.iop_id = *(jausRouter->getJausAddress());
		p_ds_discovery_client_service->pDigitalResourceDiscoveryClient_ReceiveFSM->unregisterEndpoint(endpoint, p_digital_resource_discovery_addr);
	}
}


}
