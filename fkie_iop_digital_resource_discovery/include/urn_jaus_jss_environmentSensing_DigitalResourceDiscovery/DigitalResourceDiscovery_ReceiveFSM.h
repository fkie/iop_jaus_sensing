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

#ifndef DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H
#define DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H

#include "InternalEvents/InternalEventHandler.h"
#include "JTSStateMachine.h"
#include "JausUtils.h"
#include "Transport/JausTransport.h"
#include "urn_jaus_jss_environmentSensing_DigitalResourceDiscovery/InternalEvents/InternalEventsSet.h"
#include "urn_jaus_jss_environmentSensing_DigitalResourceDiscovery/Messages/MessageSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"

#include "DigitalResourceDiscovery_ReceiveFSM_sm.h"
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_digital_resource_discovery/DigitalResourceEndpoint.h>
#include <rclcpp/rclcpp.hpp>

namespace urn_jaus_jss_environmentSensing_DigitalResourceDiscovery {

class DllExport DigitalResourceDiscovery_ReceiveFSM : public JTS::StateMachine {
public:
    DigitalResourceDiscovery_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
    virtual ~DigitalResourceDiscovery_ReceiveFSM();

    /// Handle notifications on parent state changes
    virtual void setupNotifications();
    virtual void setupIopConfiguration();

    /// Action Methods
    virtual void addAndConfirmDigitalResourceEndpointAction(RegisterDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);
    virtual void removeAndConfirmDigitalResourceEndpointAction(RemoveDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);
    virtual void reportDigitalResourceEndpointAction(QueryDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);

    /// Guard Methods

    DigitalResourceDiscovery_ReceiveFSMContext* context;

protected:
    /// References to parent FSMs
    urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
    urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

    std::shared_ptr<iop::Component> cmp;
    rclcpp::Logger logger;
    ReportDigitalResourceEndpoint p_report_digital_resource_endpoint;
    std::map<unsigned char, digital_resource_endpoint::DigitalResourceEndpoint> p_known_endpoints;
    bool pHasEndpoint(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
    unsigned char pGetFreeID();
    unsigned char pGetEndpointById(JausAddress iop_id, unsigned short int resource_id);
    int64_t p_delay_first_response;
    int64_t p_start_time;
    void p_update_report();
};

}

#endif // DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H
