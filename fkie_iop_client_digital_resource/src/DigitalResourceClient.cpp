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


#include "DigitalResourceClient.h"
#include <pluginlib/class_list_macros.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/iop_config.hpp>


using namespace iop;
using namespace urn_jaus_jss_core_Transport;
using namespace urn_jaus_jss_core_EventsClient;
using namespace urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient;

DigitalResourceClient::DigitalResourceClient()
: SlaveHandlerInterface(cmp, "DigitalResourceClientService", 2.0)
{

	p_initialized = false;
	pParentService = nullptr;
	this->m_URN = "urn:jaus:jss:environmentSensing:DigitalResourceDiscoveryClient";
	this->m_name = "DigitalResourceClientService";
	this->m_version_manjor = 2;
	this->m_version_minor = 0;
	this->m_uri_inherits_from = "urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService";
	this->m_name_inherits_from = "DigitalResourceDiscoveryClientService";
	this->m_inherits_from_version_manjor = 2;
	this->m_inherits_from_min_version_minor = 0;
}

DigitalResourceClient::~DigitalResourceClient()
{
}

void DigitalResourceClient::init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService)
{
	if (!p_initialized) {
		pParentService = static_cast<DigitalResourceDiscoveryClientService*>(parentService);
		this->p_cmp = cmp;
		this->jausRouter = jausRouter;
	}
	p_initialized = true;
}

urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService* DigitalResourceClient::getParent()
{
	return pParentService;
}

/**
 *	This is the function that will actually be run by the thread at start-up.
 *  We override it from EventReceiver in order to handle any entry
 *  actions defined by the initial state.
 */
void DigitalResourceClient::run()
{

	/// Perform any entry actions specified by the start state.
	iop::Config cfg(p_cmp, "DigitalResourceClient");
	p_pub_endoints = cfg.create_publisher<fkie_iop_msgs::msg::DigitalResourceEndpoints>("digital_endpoints", 10);
	pParentService->pDigitalResourceDiscoveryClient_ReceiveFSM->set_discovery_handler(&DigitalResourceClient::p_discovered_endpoints, this);

	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:iop:DigitalResourceDiscovery", 1, 255);
	this->set_event_name("update endpoints");
	this->set_query_before_event(true, 1.0);
	/// Kick-off the receive loop...
	EventReceiver::run();
}

bool DigitalResourceClient::processTransitions(JTS::InternalEvent* /* ie */)
{
	return false;
}
bool DigitalResourceClient::defaultTransitions(JTS::InternalEvent* /* ie */)
{
	return false;
}


void DigitalResourceClient::access_deactivated(std::string service_uri, JausAddress component)
{
	SlaveHandlerInterface::access_deactivated(service_uri, component);
	auto ros_msg = fkie_iop_msgs::msg::DigitalResourceEndpoints();
	p_pub_endoints->publish(ros_msg);
}

void DigitalResourceClient::register_events(JausAddress /* remote_addr */, double /* hz */)
{
}

void DigitalResourceClient::unregister_events(JausAddress remote_addr)
{
	stop_query(remote_addr);
}

void DigitalResourceClient::send_query(JausAddress remote_addr)
{
	pParentService->pDigitalResourceDiscoveryClient_ReceiveFSM->discoverEndpoints(remote_addr);
}

void DigitalResourceClient::stop_query(JausAddress /* remote_addr */)
{
	this->set_query_before_event(true, 1.0);
}

void DigitalResourceClient::p_discovered_endpoints(std::vector<digital_resource_endpoint::DigitalResourceEndpoint> endpoints, JausAddress& /* address */)
{
	this->set_query_before_event(false);
	auto ros_msg = fkie_iop_msgs::msg::DigitalResourceEndpoints();
	for (unsigned int i = 0; i < endpoints.size(); i++) {
		auto ep = fkie_iop_msgs::msg::DigitalResourceEndpoint();
		ep.server_url = endpoints[i].server_url;
		ep.server_type = endpoints[i].server_type;
		ep.resource_id = endpoints[i].resource_id;
		ep.address.subsystem_id = endpoints[i].iop_id.getSubsystemID();
		ep.address.node_id = endpoints[i].iop_id.getNodeID();
		ep.address.component_id = endpoints[i].iop_id.getComponentID();
		ros_msg.endpoints.push_back(ep);
	}
	p_pub_endoints->publish(ros_msg);
}

PLUGINLIB_EXPORT_CLASS(iop::DigitalResourceClient, JTS::Service)