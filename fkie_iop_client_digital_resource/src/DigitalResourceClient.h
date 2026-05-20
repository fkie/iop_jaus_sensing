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


#ifndef DIGITAL_RESOURCE_CLIENT_H
#define DIGITAL_RESOURCE_CLIENT_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "Transport/JausAddress.h"

#include "urn_jaus_jss_core_Transport/TransportService.h"
#include "urn_jaus_jss_core_EventsClient/EventsClientService.h"
#include "urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClientService.h"

#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_msgs/msg/digital_resource_endpoints.hpp>
#include <fkie_iop_component/timer.hpp>


namespace iop
{

class DigitalResourceClient : public JTS::Service, public iop::ocu::SlaveHandlerInterface
{
public:
// JTS::JausRouter* jausRouter , urn_jaus_jss_core_Transport::TransportService* pTransportService, urn_jaus_jss_core_EventsClient::EventsClientService* pEventsClientService, urn_jaus_jss_iop_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService* pDigitalResourceDiscoveryClientService
	DigitalResourceClient();
	virtual ~DigitalResourceClient();
	void init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService);
	urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService* getParent();

	virtual bool processTransitions(JTS::InternalEvent* ie);
	virtual bool defaultTransitions(JTS::InternalEvent* ie);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);
	void access_deactivated(std::string service_uri, JausAddress component);

protected:
	JTS::JausRouter* jausRouter;
	urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService* pParentService;
	std::shared_ptr<iop::Component> p_cmp;

	rclcpp::Publisher<fkie_iop_msgs::msg::DigitalResourceEndpoints>::SharedPtr p_pub_endoints;
	virtual void run();
	void p_discovered_endpoints(std::vector<digital_resource_endpoint::DigitalResourceEndpoint>, JausAddress &);

};

}

#endif // DIGITAL_RESOURCE_ENDPOINT_H
