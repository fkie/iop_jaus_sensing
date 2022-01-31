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


#include "urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/string.hpp>



using namespace JTS;

namespace urn_jaus_jss_environmentSensing_VisualSensor
{

static jUnsignedByte request_id;


VisualSensor_ReceiveFSM::VisualSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("VisualSensor"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new VisualSensor_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;

//        camera help;
//        help.id = 1000;
//        help.state = 0;
//        help.name = "Camera0";
//        help.url = "rtsp://localhost:8554/test";
//        cameraV.push_back(help);

}



VisualSensor_ReceiveFSM::~VisualSensor_ReceiveFSM()
{
	delete context;
}

void VisualSensor_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "VisualSensor_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "VisualSensor_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "VisualSensor_ReceiveFSM");

}


void VisualSensor_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "VisualSensor");
	std::vector<std::string> capabilities;
	cfg.declare_param<std::vector<std::string> >("capabilities", capabilities, false,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"A list with sensors and their capabilities. Format: ID.PARAMETER_NAME.VALUE",
		"Default: []");

	cfg.param_vector<std::vector<std::string> >("capabilities", capabilities, capabilities);
	std::map<int, std::map<std::string, std::string> > params;
	for (unsigned int i = 0; i < capabilities.size(); i++) {
		auto cap_entry = iop::split(iop::trim(capabilities[i]), '.', 3);
		if (cap_entry.size() == 3) {
			int id = std::atoi(cap_entry[0].c_str());
			std::string name = cap_entry[1];
			std::string value = cap_entry[2];
			params[id][name] = value;
		} else {
			RCLCPP_WARN(logger, "skipped capability entry '%s' because of invalid format", capabilities[i].c_str());
		}
	}
	std::map<int, std::map<std::string, std::string> >::iterator pit;
	for (pit = params.begin(); pit != params.end(); pit++) {
		std::shared_ptr<iop::VisualSensor> vs(std::make_shared<iop::VisualSensor>(cmp, pit->first, pit->second));
		vs->set_state_callback(&VisualSensor_ReceiveFSM::p_state_changed, this);
		p_sensors[pit->first] = vs;
	}
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryVisualSensorConfiguration::ID);
}

void VisualSensor_ReceiveFSM::sendConfirmSensorConfigurationAction(SetVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendConfirmSensorConfigurationAction to %s", sender.str().c_str());
	request_id = msg.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	ConfirmSensorConfiguration response;
	response.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->setRequestID(request_id);
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::sendReportSensorGeometricPropertiesAction(QuerySensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportSensorGeometricPropertiesAction to %s", sender.str().c_str());
	ReportSensorGeometricProperties response;
	std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		response.getBody()->getGeometricPropertiesList()->addElement(its->second->get_geometric());
	}
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::sendReportVisualSensorCapabilitiesAction(QueryVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportVisualSensorCapabilitiesAction to %s", sender.str().c_str());
	ReportVisualSensorCapabilities response;
	std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		response.getBody()->getVisualSensorCapabilitiesList()->addElement(its->second->get_capability());
	}
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::sendReportVisualSensorConfigurationAction(QueryVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportVisualSensorConfigurationAction to %s", sender.str().c_str());
	ReportVisualSensorConfiguration response;
	std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		response.getBody()->getVisualSensorConfigurationList()->addElement(its->second->get_configuration());
	}
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::updateVisualSensorConfigurationAction(SetVisualSensorConfiguration msg)
{
	lock_type lock(p_mutex);
	RCLCPP_DEBUG(logger, "updateVisualSensorConfigurationAction");
	request_id = msg.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	for(size_t i=0; i< msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getNumberOfElements(); i++) {
		jUnsignedByte sensorid = msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getElement(i)->getSensorID();
		std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensor> >::iterator its = p_sensors.find(sensorid);
		if (its != p_sensors.end()) {
			RCLCPP_DEBUG(logger, "  apply config to sensor ID: %d", sensorid);
			its->second->apply_cfg(msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getElement(i));
		}
	}
}



bool VisualSensor_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

void VisualSensor_ReceiveFSM::p_state_changed(jUnsignedShortInteger id)
{
	lock_type lock(p_mutex);
	ReportVisualSensorConfiguration cfg;
	std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList::VisualSensorConfigurationRec rec = its->second->get_configuration();
		RCLCPP_DEBUG(logger, "  apply config to sensor ID: %d, power_state: %d, set: %d", (int) its->second->get_id(), rec.getSensorState(), (int)its->second->get_switch_state());
		cfg.getBody()->getVisualSensorConfigurationList()->addElement(rec);
	}
	p_configuration = cfg;
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryVisualSensorConfiguration::ID, &p_configuration);
}

}
