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
#include <fkie_iop_component/iop_config.h>




using namespace JTS;

namespace urn_jaus_jss_environmentSensing_VisualSensor
{

static jUnsignedByte request_id;


VisualSensor_ReceiveFSM::VisualSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new VisualSensor_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;

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
	iop::Config cfg("~VisualSensor");
	XmlRpc::XmlRpcValue sensor_names;
	cfg.param("sensor_names", sensor_names, sensor_names);
	if (sensor_names.valid()) {
		ROS_WARN("'~sensor_names' is depricated, use '~capabilities' instead");
		// parse the parameter into a map
		for(int i = 0; i < sensor_names.size(); i++) {
			if (sensor_names[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
				for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = sensor_names[i].begin(); iterator != sensor_names[i].end(); iterator++) {
					int ep_id = std::atoi(iterator->first.c_str());
					std::string vi_name = iterator->second;
					boost::shared_ptr<iop::VisualSensor> vs(boost::make_shared<iop::VisualSensor>(ep_id, vi_name));
					p_sensors[ep_id] = vs;
					ROS_INFO_NAMED("VisualSensor", "  %d: %s", ep_id, vi_name.c_str());
				}
			} else {
				ROS_ERROR("wrong entry of '~sensor_names' format, expected list of: ID: NAME");
			}
		}
	}
	// read sensor configuration
	XmlRpc::XmlRpcValue caps;
	cfg.param("capabilities", caps, caps);
	if (!caps.valid()) {
		ROS_ERROR("wrong '~capabilities' format, expected dict of: ID: [name, zoom_mode, position, ...]");
		ROS_BREAK();
	}
	// parse the parameter into a map
	for(int i = 0; i < caps.size(); i++) {
		if (caps[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = caps[i].begin(); iterator != caps[i].end(); iterator++) {
				int id = std::atoi(iterator->first.c_str());
				boost::shared_ptr<iop::VisualSensor> vs(boost::make_shared<iop::VisualSensor>(id, iterator->second));
				vs->set_state_callback(&VisualSensor_ReceiveFSM::p_state_changed, this);
				p_sensors[id] = vs;
			}
		} else {
			ROS_ERROR("wrong entry of '~sensor_names' format, expected list of: ID: NAME");
		}
	}
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryVisualSensorConfiguration::ID);
}

void VisualSensor_ReceiveFSM::sendConfirmSensorConfigurationAction(SetVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("VisualSensor", "sendConfirmSensorConfigurationAction to %s", sender.str().c_str());
	request_id = msg.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	ConfirmSensorConfiguration response;
	response.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->setRequestID(request_id);
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::sendReportSensorGeometricPropertiesAction(QuerySensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("VisualSensor", "sendReportSensorGeometricPropertiesAction to %s", sender.str().c_str());
	ReportSensorGeometricProperties response;
	std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		response.getBody()->getGeometricPropertiesList()->addElement(its->second->get_geometric());
	}
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::sendReportVisualSensorCapabilitiesAction(QueryVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("VisualSensor", "sendReportVisualSensorCapabilitiesAction to %s", sender.str().c_str());
	ReportVisualSensorCapabilities response;
	std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		response.getBody()->getVisualSensorCapabilitiesList()->addElement(its->second->get_capability());
	}
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::sendReportVisualSensorConfigurationAction(QueryVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("VisualSensor", "sendReportVisualSensorConfigurationAction to %s", sender.str().c_str());
	ReportVisualSensorConfiguration response;
	std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		response.getBody()->getVisualSensorConfigurationList()->addElement(its->second->get_configuration());
	}
	sendJausMessage(response, sender);
}

void VisualSensor_ReceiveFSM::updateVisualSensorConfigurationAction(SetVisualSensorConfiguration msg)
{
	lock_type lock(p_mutex);
	ROS_DEBUG_NAMED("VisualSensor", "updateVisualSensorConfigurationAction");
	request_id = msg.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	for(size_t i=0; i< msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getNumberOfElements(); i++) {
		jUnsignedByte sensorid = msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getElement(i)->getSensorID();
		std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensor> >::iterator its = p_sensors.find(sensorid);
		if (its != p_sensors.end()) {
			ROS_DEBUG_NAMED("VisualSensor", "  apply config to sensor ID: %d", sensorid);
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
	std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensor> >::iterator its;
	for (its = p_sensors.begin(); its != p_sensors.end(); its++) {
		ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList::VisualSensorConfigurationRec rec = its->second->get_configuration();
		ROS_DEBUG_NAMED("VisualSensor", "  apply config to sensor ID: %d, power_state: %d, set: %d", (int) its->second->get_id(), rec.getSensorState(), (int)its->second->get_switch_state());
		cfg.getBody()->getVisualSensorConfigurationList()->addElement(rec);
	}
	p_configuration = cfg;
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryVisualSensorConfiguration::ID, &p_configuration);
}

};
