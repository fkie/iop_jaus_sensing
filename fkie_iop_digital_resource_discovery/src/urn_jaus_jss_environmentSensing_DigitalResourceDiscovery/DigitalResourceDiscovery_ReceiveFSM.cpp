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
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#include "urn_jaus_jss_environmentSensing_DigitalResourceDiscovery/DigitalResourceDiscovery_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>

using namespace JTS;
using namespace digital_resource_endpoint;

namespace urn_jaus_jss_environmentSensing_DigitalResourceDiscovery {

DigitalResourceDiscovery_ReceiveFSM::DigitalResourceDiscovery_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
    : logger(cmp->get_logger().get_child("DigitalResourceDiscovery"))
{

    /*
     * If there are other variables, context must be constructed last so that all
     * class variables are available if an EntryAction of the InitialState of the
     * statemachine needs them.
     */
    context = new DigitalResourceDiscovery_ReceiveFSMContext(*this);

    this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
    this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
    this->cmp = cmp;
    p_delay_first_response = 5;
    p_start_time = 0;
}

DigitalResourceDiscovery_ReceiveFSM::~DigitalResourceDiscovery_ReceiveFSM()
{
    delete context;
}

void DigitalResourceDiscovery_ReceiveFSM::setupNotifications()
{
    pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DigitalResourceDiscovery_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
    pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DigitalResourceDiscovery_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
    registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "DigitalResourceDiscovery_ReceiveFSM");
    registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "DigitalResourceDiscovery_ReceiveFSM");
}

void DigitalResourceDiscovery_ReceiveFSM::setupIopConfiguration()
{
    iop::Config cfg(cmp, "DigitalResourceDiscovery");
    cfg.param<int64_t>("delay_first_response", p_delay_first_response, p_delay_first_response, true,
        rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
        "Waits given time in seconds for start of other components. While this time all requests for DigitalResourceEndpoint will be ignored.",
        "Default: 5 sec");
    p_start_time = iop::Component::now_secs();
    pEvents_ReceiveFSM->get_event_handler().register_query(QueryDigitalResourceEndpoint::ID);
    pEvents_ReceiveFSM->get_event_handler().set_report(QueryDigitalResourceEndpoint::ID, &p_report_digital_resource_endpoint);
}

void DigitalResourceDiscovery_ReceiveFSM::addAndConfirmDigitalResourceEndpointAction(RegisterDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{
    JausAddress sender = transportData.getAddress();
    RegisterDigitalResourceEndpoint::Body::RegisterDigitalResourceSeq* regmsg = msg.getBody()->getRegisterDigitalResourceSeq();
    unsigned char request_id = regmsg->getRequestIDRec()->getRequestID();
    unsigned char server_type = regmsg->getDigitalResourceEndpointRec()->getServerType();
    std::string server_url = regmsg->getDigitalResourceEndpointRec()->getServerURL();
    unsigned short int resource_id = regmsg->getDigitalResourceEndpointRec()->getResourceID();
    RCLCPP_DEBUG(logger, "addAndConfirmDigitalResourceEndpointAction %s, request_id: %d, server_type: %d, server_url: %s",
        sender.str().c_str(), request_id, server_type, server_url.c_str());
    RegisterDigitalResourceEndpoint::Body::RegisterDigitalResourceSeq::DigitalResourceEndpointRec::JAUS_ID* iop_id_rec = regmsg->getDigitalResourceEndpointRec()->getJAUS_ID();
    JausAddress iop_id(iop_id_rec->getSubsystemID(), iop_id_rec->getNodeID(), iop_id_rec->getComponentID());
    DigitalResourceEndpoint endpoint(server_type, server_url, iop_id, resource_id, request_id);
    unsigned char index_id = pGetEndpointById(iop_id, resource_id);
    if (index_id == 0) {
        index_id = pGetFreeID();
    }
    if (index_id < 255) {
        p_known_endpoints[index_id] = endpoint;
        ConfirmDigitalResourceEndpoint confirm_msg;
        confirm_msg.getBody()->getConfirmDigitalResourceEndpointRec()->setID(index_id);
        confirm_msg.getBody()->getConfirmDigitalResourceEndpointRec()->setRequestID(request_id);
        sendJausMessage(confirm_msg, sender);
        p_update_report();
    }
}

void DigitalResourceDiscovery_ReceiveFSM::removeAndConfirmDigitalResourceEndpointAction(RemoveDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{

    JausAddress sender = transportData.getAddress();
    RemoveDigitalResourceEndpoint::Body::RemoveDigitalResourceEndpointRec* remmsg = msg.getBody()->getRemoveDigitalResourceEndpointRec();
    unsigned char id = remmsg->getID();
    unsigned char request_id = remmsg->getRequestID();
    RCLCPP_DEBUG(logger, "removeAndConfirmDigitalResourceEndpointAction %s, id_to_remove: %d, request_id: %d",
        sender.str().c_str(), id, request_id);
    std::map<unsigned char, DigitalResourceEndpoint>::iterator it = p_known_endpoints.find(id);
    if (it != p_known_endpoints.end()) {
        ConfirmDigitalResourceEndpoint confirm_msg;
        confirm_msg.getBody()->getConfirmDigitalResourceEndpointRec()->setID(id);
        confirm_msg.getBody()->getConfirmDigitalResourceEndpointRec()->setRequestID(request_id);
        p_known_endpoints.erase(it);
        sendJausMessage(confirm_msg, sender);
        p_update_report();
    }
}

void DigitalResourceDiscovery_ReceiveFSM::reportDigitalResourceEndpointAction(QueryDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{
    if (p_start_time > 0 and iop::Component::now_secs() - p_start_time < p_delay_first_response) {
        // wait for start of other component, skip response in first seconds
        return;
    }
    JausAddress sender = transportData.getAddress();
    RCLCPP_DEBUG(logger, "reportDigitalResourceEndpointAction %s", sender.str().c_str());
    ReportDigitalResourceEndpoint response;
    std::map<unsigned char, DigitalResourceEndpoint>::iterator it;
    for (it = p_known_endpoints.begin(); it != p_known_endpoints.end(); it++) {
        ReportDigitalResourceEndpoint::Body::DigitalResourceEndpointList::DigitalResourceEndpointRec rpoint;
        rpoint.setServerType(it->second.server_type);
        rpoint.setServerURL(it->second.server_url);
        rpoint.setResourceID(it->second.resource_id);
        ReportDigitalResourceEndpoint::Body::DigitalResourceEndpointList::DigitalResourceEndpointRec::JAUS_ID iop_id_rec;
        iop_id_rec.setSubsystemID(it->second.iop_id.getSubsystemID());
        iop_id_rec.setNodeID(it->second.iop_id.getNodeID());
        iop_id_rec.setComponentID(it->second.iop_id.getComponentID());
        rpoint.setJAUS_ID(iop_id_rec);
        response.getBody()->getDigitalResourceEndpointList()->addElement(rpoint);
    }
    sendJausMessage(response, sender);
}

bool DigitalResourceDiscovery_ReceiveFSM::pHasEndpoint(DigitalResourceEndpoint endpoint)
{
    std::map<unsigned char, DigitalResourceEndpoint>::iterator it;
    for (it = p_known_endpoints.begin(); it != p_known_endpoints.end(); it++) {
        if (it->second == endpoint) {
            return true;
        }
    }
    return false;
}

unsigned char DigitalResourceDiscovery_ReceiveFSM::pGetEndpointById(JausAddress iop_id, unsigned short int resource_id)
{
    std::map<unsigned char, DigitalResourceEndpoint>::iterator it;
    for (it = p_known_endpoints.begin(); it != p_known_endpoints.end(); it++) {
        if (it->second.iop_id.get() == iop_id.get() and it->second.resource_id == resource_id) {
            return it->first;
        }
    }
    return 0;
}

unsigned char DigitalResourceDiscovery_ReceiveFSM::pGetFreeID()
{
    jUnsignedByte result = 1;
    while (result < 255) {
        if (p_known_endpoints.find(result) != p_known_endpoints.end()) {
            result++;
        } else {
            break;
        }
    }
    return result;
}

void DigitalResourceDiscovery_ReceiveFSM::p_update_report()
{
    while (p_report_digital_resource_endpoint.getBody()->getDigitalResourceEndpointList()->getNumberOfElements() > 0) {
        p_report_digital_resource_endpoint.getBody()->getDigitalResourceEndpointList()->deleteLastElement();
    }
    std::map<unsigned char, DigitalResourceEndpoint>::iterator it;
    for (it = p_known_endpoints.begin(); it != p_known_endpoints.end(); it++) {
        ReportDigitalResourceEndpoint::Body::DigitalResourceEndpointList::DigitalResourceEndpointRec rpoint;
        rpoint.setServerType(it->second.server_type);
        rpoint.setServerURL(it->second.server_url);
        rpoint.setResourceID(it->second.resource_id);
        ReportDigitalResourceEndpoint::Body::DigitalResourceEndpointList::DigitalResourceEndpointRec::JAUS_ID iop_id_rec;
        iop_id_rec.setSubsystemID(it->second.iop_id.getSubsystemID());
        iop_id_rec.setNodeID(it->second.iop_id.getNodeID());
        iop_id_rec.setComponentID(it->second.iop_id.getComponentID());
        rpoint.setJAUS_ID(iop_id_rec);
        p_report_digital_resource_endpoint.getBody()->getDigitalResourceEndpointList()->addElement(rpoint);
    }
    pEvents_ReceiveFSM->get_event_handler().set_report(QueryDigitalResourceEndpoint::ID, &p_report_digital_resource_endpoint);
}

}
