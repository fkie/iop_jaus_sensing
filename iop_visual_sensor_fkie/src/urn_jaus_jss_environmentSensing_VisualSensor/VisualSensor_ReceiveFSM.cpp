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
#include <iop_component_fkie/iop_config.h>




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
	if (!sensor_names.valid()) {
		ROS_ERROR("wrong '~sensor_names' format, expected list of: ID: NAME");
		ROS_BREAK();
	}
	// parse the parameter into a map
	for(int i = 0; i < sensor_names.size(); i++) {
		if (sensor_names[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = sensor_names[i].begin(); iterator != sensor_names[i].end(); iterator++) {
				int ep_id = std::atoi(iterator->first.c_str());
				std::string vi_name = iterator->second;
				p_sensor_names[ep_id] = vi_name;
				ROS_INFO_NAMED("VisualSensor", "  %d: %s", ep_id, vi_name.c_str());
			}
		} else {
			ROS_ERROR("wrong entry of '~sensor_names' format, expected list of: ID: NAME");
		}
	}
}

void VisualSensor_ReceiveFSM::sendConfirmSensorConfigurationAction(SetVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("VisualSensor", "sendConfirmSensorConfigurationAction to %d.%d.%d", subsystem_id, node_id, component_id);
	request_id = msg.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->getRequestID();
	ConfirmSensorConfiguration response;
	response.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->setRequestID(request_id);
	sendJausMessage(response,sender);
}

void VisualSensor_ReceiveFSM::sendReportSensorGeometricPropertiesAction(QuerySensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("VisualSensor", "sendReportSensorGeometricPropertiesAction to %d.%d.%d (default cfg)", subsystem_id, node_id, component_id);
	ReportSensorGeometricProperties response;
	sendJausMessage(response,sender);
}

void VisualSensor_ReceiveFSM::sendReportVisualSensorCapabilitiesAction(QueryVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("VisualSensor", "sendReportVisualSensorCapabilitiesAction to %d.%d.%d (default cfg)", subsystem_id, node_id, component_id);
	ReportVisualSensorCapabilities response;
	std::map<unsigned short, std::string>::iterator it;
	for (it = p_sensor_names.begin(); it != p_sensor_names.end(); it++) {
		ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec entry;
		entry.setSensorID(it->first);
		entry.setSensorName(it->second);
		response.getBody()->getVisualSensorCapabilitiesList()->addElement(entry);
	}
	sendJausMessage(response,sender);
}

void VisualSensor_ReceiveFSM::sendReportVisualSensorConfigurationAction(QueryVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("VisualSensor", "sendReportVisualSensorConfigurationAction to %d.%d.%d (default cfg)", subsystem_id, node_id, component_id);
	ReportVisualSensorConfiguration response;
	std::map<unsigned short, std::string>::iterator it;
	for (it = p_sensor_names.begin(); it != p_sensor_names.end(); it++) {
		ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList::VisualSensorConfigurationRec entry;
		entry.setSensorID(it->first);
		response.getBody()->getVisualSensorConfigurationList()->addElement(entry);
	}
	sendJausMessage(response,sender);
}

void VisualSensor_ReceiveFSM::updateVisualSensorConfigurationAction(SetVisualSensorConfiguration msg)
{
	/// Insert User Code HERE
	ROS_DEBUG_NAMED("VisualSensor", "updateVisualSensorConfigurationAction");
	request_id = msg.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->getRequestID();

	for(size_t i=0; i< msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getNumberOfElements(); i++) {
		jUnsignedByte sensorid = msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getElement(i)->getSensorID();
		printf("[VisualSensor]   Sensor ID: %d\n", sensorid);
		//    std::vector<camera>::iterator it;
		//    bool found = false;
		//    for(it = this->cameraV.begin(); it != this->cameraV.end(); it++)
		//    {
		//      if(it->id == sensorid)
		//      {
		//        it->state = msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getElement(i)->getSensorState();
		//        found = true;
		//        std::cout << "found sensor id: " << msg.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->getElement(i)->getSensorID() << "\n";
		//      }
		//    }
		//    if(!found)
		//    {
		//      ConfirmSensorConfiguration::Body::ConfirmSensorConfigurationSequence::ConfirmSensorList::ConfirmSensorConfigurationVariant data;
		//      data.getSensorIdRec()->setSensorID(sensorid);
		//      data.getStillImageSensorErrorRec()->setErrorMessage("Unknown SensorID");
		//      data.getStillImageSensorErrorRec()->setStillImageErrorCode(0);
		//      this->errorList.addElement(data);
		//    }
	}
}



bool VisualSensor_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}


};
