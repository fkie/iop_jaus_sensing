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

#include <algorithm>

#include "urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/iop_config.hpp>

using namespace JTS;
using namespace digital_resource_endpoint;

namespace urn_jaus_jss_environmentSensing_DigitalResourceDiscoveryClient {

DigitalResourceDiscoveryClient_ReceiveFSM::DigitalResourceDiscoveryClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
    : logger(cmp->get_logger().get_child("DigitalResourceDiscoveryClient"))
{

    /*
     * If there are other variables, context must be constructed last so that all
     * class variables are available if an EntryAction of the InitialState of the
     * statemachine needs them.
     */
    context = new DigitalResourceDiscoveryClient_ReceiveFSMContext(*this);

    this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
    this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
    this->cmp = cmp;
    p_request_id = 0;
    p_enable_ros_interface = false;
}

DigitalResourceDiscoveryClient_ReceiveFSM::~DigitalResourceDiscoveryClient_ReceiveFSM()
{
    delete context;
}

void DigitalResourceDiscoveryClient_ReceiveFSM::setupNotifications()
{
    pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DigitalResourceDiscoveryClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
    pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DigitalResourceDiscoveryClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
    registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "DigitalResourceDiscoveryClient_ReceiveFSM");
    registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "DigitalResourceDiscoveryClient_ReceiveFSM");
}

void DigitalResourceDiscoveryClient_ReceiveFSM::setupIopConfiguration()
{
    iop::Config cfg(cmp, "DigitalResourceDiscoveryClient");
    if (p_enable_ros_interface) {
        p_pub_endoints = cfg.create_publisher<fkie_iop_msgs::msg::DigitalResourceEndpoints>("endpoints", 1);
        p_srv_update_endpoints = cfg.create_service<fkie_iop_msgs::srv::QueryByAddr>("update_endpoints", std::bind(&DigitalResourceDiscoveryClient_ReceiveFSM::pUpdateEndpointsSrv, this, std::placeholders::_1, std::placeholders::_2));
    }
}

bool DigitalResourceDiscoveryClient_ReceiveFSM::pUpdateEndpointsSrv(const fkie_iop_msgs::srv::QueryByAddr::Request::SharedPtr req, fkie_iop_msgs::srv::QueryByAddr::Response::SharedPtr res)
{
    if (p_enable_ros_interface) {
        RCLCPP_DEBUG(logger, "update endpoints of %d.%d.%d", req->address.subsystem_id, req->address.node_id, req->address.component_id);
        discoverEndpoints(JausAddress(req->address.subsystem_id, req->address.node_id, req->address.component_id));
        return true;
    }
    return false;
}

void DigitalResourceDiscoveryClient_ReceiveFSM::confirmDigitalResourceEndpointAction(ConfirmDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    bool confirmed = false;
    unsigned char server_id = msg.getBody()->getConfirmDigitalResourceEndpointRec()->getID();
    unsigned char request_id = msg.getBody()->getConfirmDigitalResourceEndpointRec()->getRequestID();
    // check, is it a registration confirmation
    for (std::deque<DigitalResourceEndpoint>::iterator it = p_toregister_endpoints.begin(); it != p_toregister_endpoints.end(); ++it) {
        if (it->request_id == request_id) {
            DigitalResourceEndpoint reg = *it;
            reg.request_id = server_id;
            p_registered_endpoints.push_back(reg);
            p_toregister_endpoints.erase(it);
            RCLCPP_INFO(logger, "registration for %s confirmed!", reg.server_url.c_str());
            confirmed = true;
            break;
        }
    }
    // check, is it a unregistration confirmation
    for (std::deque<DigitalResourceEndpoint>::iterator it = p_tounregister_endpoints.begin(); it != p_tounregister_endpoints.end(); ++it) {
        if (it->request_id == request_id) {
            DigitalResourceEndpoint reg = *it;
            p_tounregister_endpoints.erase(it);
            RCLCPP_INFO(logger, "unregistration for %s confirmed!", reg.server_url.c_str());
            confirmed = true;
            break;
        }
    }
    if (!confirmed) {
        RCLCPP_WARN(logger, "received confirmation for wrong request_id: %d", request_id);
    }
}

void DigitalResourceDiscoveryClient_ReceiveFSM::reportDigitalResourceEndpointAction(ReportDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    uint16_t subsystem_id = transportData.getSrcSubsystemID();
    uint8_t node_id = transportData.getSrcNodeID();
    uint8_t component_id = transportData.getSrcComponentID();
    JausAddress sender(subsystem_id, node_id, component_id);
    RCLCPP_DEBUG(logger, "reportDigitalResourceEndpointAction from %d.%d.%d",
        subsystem_id, node_id, component_id);
    ReportDigitalResourceEndpoint::Body::DigitalResourceEndpointList* endpoints = msg.getBody()->getDigitalResourceEndpointList();
    std::vector<digital_resource_endpoint::DigitalResourceEndpoint> v_endpoints;
    for (unsigned int ei = 0; ei < endpoints->getNumberOfElements(); ei++) {
        DigitalResourceEndpoint endpoint;
        ReportDigitalResourceEndpoint::Body::DigitalResourceEndpointList::DigitalResourceEndpointRec* ep = endpoints->getElement(ei);
        endpoint.server_url = ep->getServerURL();
        endpoint.server_type = ep->getServerType();
        endpoint.resource_id = ep->getResourceID();
        endpoint.iop_id = JausAddress(ep->getJAUS_ID()->getSubsystemID(), ep->getJAUS_ID()->getNodeID(), ep->getJAUS_ID()->getComponentID());
        v_endpoints.push_back(endpoint);
    }
    auto ros_msg = fkie_iop_msgs::msg::DigitalResourceEndpoints();
    for (unsigned int i = 0; i < v_endpoints.size(); i++) {
        auto ep = fkie_iop_msgs::msg::DigitalResourceEndpoint();
        ep.server_url = v_endpoints[i].server_url;
        ep.server_type = v_endpoints[i].server_type;
        ep.resource_id = v_endpoints[i].resource_id;
        ep.address.subsystem_id = v_endpoints[i].iop_id.getSubsystemID();
        ep.address.node_id = v_endpoints[i].iop_id.getNodeID();
        ep.address.component_id = v_endpoints[i].iop_id.getComponentID();
        ros_msg.endpoints.push_back(ep);
    }
    if (p_enable_ros_interface) {
        p_pub_endoints->publish(ros_msg);
    }
    if (class_discovery_callback_ != nullptr) {
        class_discovery_callback_(v_endpoints, sender);
    }
}

void DigitalResourceDiscoveryClient_ReceiveFSM::registerEndpoint(DigitalResourceEndpoint endpoint, const JausAddress digital_resource_discovery_service)
{
    //	if (pHasEndpoint(endpoint)) {
    //		RCLCPP_WARN(logger, "endpoint %s already registered, skip registration", endpoint.server_url.c_str());
    //		return;
    //	}
    RCLCPP_INFO(logger, "register endpoint: %s", endpoint.server_url.c_str());
    // register the digital video endpoint by DigitalResourceDiscovery service
    RegisterDigitalResourceEndpoint msg;
    endpoint.request_id = p_request_id;
    p_request_id++;
    msg.getBody()->getRegisterDigitalResourceSeq()->getRequestIDRec()->setRequestID(endpoint.request_id);
    RegisterDigitalResourceEndpoint::Body::RegisterDigitalResourceSeq::DigitalResourceEndpointRec* digital_resource;
    digital_resource = msg.getBody()->getRegisterDigitalResourceSeq()->getDigitalResourceEndpointRec();
    digital_resource->setServerType(endpoint.server_type);
    digital_resource->setServerURL(endpoint.server_url);
    RegisterDigitalResourceEndpoint::Body::RegisterDigitalResourceSeq::DigitalResourceEndpointRec::JAUS_ID iop_id;
    iop_id.setSubsystemID(endpoint.iop_id.getSubsystemID());
    iop_id.setNodeID(endpoint.iop_id.getNodeID());
    iop_id.setComponentID(endpoint.iop_id.getComponentID());
    digital_resource->setJAUS_ID(iop_id);
    digital_resource->setResourceID(endpoint.resource_id);
    if (std::find(p_toregister_endpoints.begin(), p_toregister_endpoints.end(), endpoint) == p_toregister_endpoints.end()) {
        p_toregister_endpoints.push_back(endpoint);
    }
    sendJausMessage(msg, digital_resource_discovery_service);
}

void DigitalResourceDiscoveryClient_ReceiveFSM::unregisterEndpoint(DigitalResourceEndpoint endpoint, const JausAddress digital_resource_discovery_service)
{
    std::deque<DigitalResourceEndpoint>::iterator deg = std::find(p_registered_endpoints.begin(), p_registered_endpoints.end(), endpoint);
    if (deg == p_registered_endpoints.end()) {
        RCLCPP_WARN(logger, "endpoint %s is not registered, skip unregistration", endpoint.server_url.c_str());
        return;
    }
    RCLCPP_INFO(logger, "unregister endpoint: %s", endpoint.server_url.c_str());
    // unregister the digital video endpoint by DigitalResourceDiscovery service
    RemoveDigitalResourceEndpoint msg;
    msg.getBody()->getRemoveDigitalResourceEndpointRec()->setID(deg->request_id);
    deg->request_id = p_request_id;
    p_request_id++;
    msg.getBody()->getRemoveDigitalResourceEndpointRec()->setRequestID(deg->request_id);
    p_tounregister_endpoints.push_back(*deg);
    p_registered_endpoints.erase(deg);
    sendJausMessage(msg, digital_resource_discovery_service);
}

void DigitalResourceDiscoveryClient_ReceiveFSM::discoverEndpoints(const JausAddress digital_resource_discovery_service)
{
    QueryDigitalResourceEndpoint qmsg;
    sendJausMessage(qmsg, digital_resource_discovery_service);
}

bool DigitalResourceDiscoveryClient_ReceiveFSM::pHasEndpoint(digital_resource_endpoint::DigitalResourceEndpoint& endpoint)
{
    return (std::find(p_registered_endpoints.begin(), p_registered_endpoints.end(), endpoint) != p_registered_endpoints.end());
}

}
