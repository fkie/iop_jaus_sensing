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

#include "urn_jaus_jss_environmentSensing_RangeSensor/RangeSensor_ReceiveFSM.h"
#include <iostream>
#include <string>
#include <libgen.h>
#include "include/Iop_range_sensor_fkie.h"
#include "JausUtils.h"
#include <sensor_msgs/LaserScan.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iop_builder_fkie/timestamp.h>


using namespace JTS;
using namespace urn_jaus_jss_core_Transport;
using namespace urn_jaus_jss_core_Events;
using namespace urn_jaus_jss_core_AccessControl;
using namespace urn_jaus_jss_environmentSensing_RangeSensor;



using namespace JTS;

namespace urn_jaus_jss_environmentSensing_RangeSensor
{



RangeSensor_ReceiveFSM::RangeSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new RangeSensor_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	p_tf_frame_robot = "base_link";
}



RangeSensor_ReceiveFSM::~RangeSensor_ReceiveFSM()
{
	stop_subscriber();
	delete context;
}

void RangeSensor_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_RangeSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_RangeSensor_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_RangeSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_RangeSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "RangeSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "RangeSensor_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "RangeSensor_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "RangeSensor_ReceiveFSM");

	stop_subscriber();
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	pnh.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	ROS_INFO("tf_frame_robot: %s", p_tf_frame_robot.c_str());
	XmlRpc::XmlRpcValue v;
	int sensor_id = 0;
	pnh.param("sensors", v, v);
	ROS_INFO("Used sensors:");
	p_mutex.lock();
	if (v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		for(unsigned int i = 0; i < v.size(); i++) {
			std::string ros_topic = v[i];
			// resolve to node namespace
			std::string ros_topic_resolved = ros::names::resolve(ros_topic);
			ROS_INFO("  create subscriber for %s", ros_topic_resolved.c_str());
			RangeSensor *sensor = new RangeSensor(sensor_id, ros_topic_resolved, this);
			sensor_id++;
			p_sensors.push_back(sensor);
		}
	} else {
		ROS_WARN("wrong ~sensors parameter type! It should be an array with format [ros_topic, ...]");
	}
	p_mutex.unlock();
}

RangeSensor_ReceiveFSM::RangeSensor::RangeSensor(int id, std::string topic, RangeSensor_ReceiveFSM *parent)
{
	this->id = id;
	this->initialized = false;
	this->ros_topic = topic;
	ros::NodeHandle nh;
	this->ros_sub = nh.subscribe(topic, 1, &RangeSensor_ReceiveFSM::scan_callback, parent);
	ReportRangeSensorCapabilities::Body::RangeSensorCapabilitiesList::RangeSensorCapabilitiesRec caprec;
	caprec.setSensorID(id);
	caprec.setSensorName(std::string(basename((char *)topic.c_str())));
	caprec.getSupportedCompression()->setNoCompression(1);
	this->capabilities.getBody()->getRangeSensorCapabilitiesList()->addElement(caprec);
	ReportRangeSensorConfiguration::Body::RangeSensorConfigurationList::RangeSensorConfigurationRec cfgrec;
	cfgrec.setSensorID(id);
	// set to no geometric properties
	ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence geosec;
	geosec.getSensorIdRec()->setSensorID(id);
	ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence::GeometricPropertiesVariant geovar;
	ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence::GeometricPropertiesVariant::NoGeometricPropertiesVariant nogeo;
	geovar.setNoGeometricPropertiesVariant(nogeo);
	geosec.setGeometricPropertiesVariant(geovar);
	this->geometric.getBody()->getGeometricPropertiesList()->addElement(geosec);
	// set state to Standby. If the sensor is standby on receiving a laser scan the configuration will be updated.
	cfgrec.setSensorState(1);
	configuration.getBody()->getRangeSensorConfigurationList()->addElement(cfgrec);
	ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant datavar;
	datavar.getRangeSensorDataErrorRec()->setSensorID(id);
	datavar.getRangeSensorDataErrorRec()->setDataErrorCode(0);
	sensor_data.getBody()->getRangeSensorDataList()->addElement(datavar);
}

RangeSensor_ReceiveFSM::RangeSensor::~RangeSensor()
{
	ros_sub.shutdown();
}


void RangeSensor_ReceiveFSM::stop_subscriber()
{
	for (unsigned int i = 0; i < p_sensors.size(); i++) {
		delete p_sensors[i];
	}
	p_sensors.clear();
}

void RangeSensor_ReceiveFSM::scan_callback(const ros::MessageEvent<sensor_msgs::LaserScan const>& event)
{
	ros::M_string& header = event.getConnectionHeader();
	ros::Time receipt_time = event.getReceiptTime();
	const sensor_msgs::LaserScan::ConstPtr &msg = event.getMessage();
	RangeSensor *sensor = NULL;
	for (unsigned int i = 0; i < p_sensors.size(); i++) {
		if (p_sensors[i]->ros_topic.compare(header["topic"]) == 0) {
			sensor = p_sensors[i];
			break;
		}
	}
	if (sensor != NULL) {
		if (!sensor->initialized) {
			// try to get the robot coordinate system
			bool geo_init = false;
			try {
				if (!p_tf_frame_robot.empty()) {
					tfListener.waitForTransform(p_tf_frame_robot, msg->header.frame_id, ros::Time(0), ros::Duration(0.3));
					tf::StampedTransform transform;
					tfListener.lookupTransform(p_tf_frame_robot, msg->header.frame_id, ros::Time(0), transform);
					ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence::GeometricPropertiesVariant::StaticGeometricPropertiesRec staticgeo;
					staticgeo.getSensorPosition()->setPositionVectorElement(0, transform.getOrigin().x());
					staticgeo.getSensorPosition()->setPositionVectorElement(1, transform.getOrigin().y());
					staticgeo.getSensorPosition()->setPositionVectorElement(2, transform.getOrigin().z());
					staticgeo.getUnitQuaternion()->setUnitQuaternionElement(0, transform.getRotation().getX());
					staticgeo.getUnitQuaternion()->setUnitQuaternionElement(1, transform.getRotation().getY());
					staticgeo.getUnitQuaternion()->setUnitQuaternionElement(2, transform.getRotation().getZ());
					staticgeo.getUnitQuaternion()->setUnitQuaternionElement(3, transform.getRotation().getW());
					sensor->geometric.getBody()->getGeometricPropertiesList()->getElement(0)->getGeometricPropertiesVariant()->setStaticGeometricPropertiesRec(staticgeo);
					sensor->geometric.getBody()->getGeometricPropertiesList()->getElement(0)->getGeometricPropertiesVariant()->setFieldValue(1);
					geo_init = true;
				}
			} catch (tf::TransformException &ex){
				ROS_WARN_STREAM_THROTTLE(1.0, "Could not lookup transform from " << p_tf_frame_robot << " to " << msg->header.frame_id << ": " << ex.what());
			}
			sensor->capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMinimumHorizontalFieldOfViewStartAngle(msg->angle_min);
			sensor->capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMaximumHorizontalFieldOfViewStopAngle(msg->angle_max);
			sensor->capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMinimumRange(msg->range_min);
			sensor->capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMaximumRange(msg->range_max);
			sensor->configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setSensorState(0);
			sensor->configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setHorizontalFieldOfViewStartAngle(msg->angle_min);
			sensor->configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setHorizontalFieldOfViewStopAngle(msg->angle_max);
			sensor->configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setMinimumRange(msg->range_min);
			sensor->configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setMaximumRange(msg->range_max);
			if (geo_init) {
				sensor->initialized = true;
			}
		}
		if (sensor->sensor_data.getBody()->getRangeSensorDataList()->getNumberOfElements() > 0)
		{
			sensor->sensor_data.getBody()->getRangeSensorDataList()->deleteLastElement();
		}
		ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant sdatavar;
		// set timestamp
		ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataRec::TimeStamp ts;
		// current date/time based on current system
		iop::Timestamp stamp(msg->header.stamp);
		ts.setDay(stamp.days);
		ts.setHour(stamp.hours);
		ts.setMinutes(stamp.minutes);
		ts.setSeconds(stamp.seconds);
		ts.setMilliseconds(stamp.milliseconds);
		sdatavar.getRangeSensorDataSeq()->getRangeSensorDataRec()->setSensorID(sensor->id);
		sdatavar.getRangeSensorDataSeq()->getRangeSensorDataRec()->setTimeStamp(ts);
		sdatavar.getRangeSensorDataSeq()->getRangeSensorDataRec()->setReportCoordinateSystem(0);
		// set points
		ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataPointList::RangeSensorDataPointRec point;
		for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
			point.setBearing(msg->angle_min + (i * msg->angle_increment));
			if (std::isnan(msg->ranges[i])) {
				point.setRangeValidity(0);
			} else {
				point.setRange(msg->ranges[i]);
				point.setRangeValidity(1);
			}
			point.setInclination(0.0);
			sdatavar.getRangeSensorDataSeq()->getRangeSensorDataPointList()->addElement(point);
		}
		sdatavar.setFieldValue(1);
		sensor->sensor_data.getBody()->getRangeSensorDataList()->addElement(sdatavar);
		this->pEvents_ReceiveFSM->set_event_report(0x2803, sensor->sensor_data);
	}
}

bool p_requested_capability(QueryRangeSensorCapabilities msg, int id)
{
	if (msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements() == 0) {
		return true;
	}
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements(); i++) {
		if (msg.getBody()->getRangeSensorCapabilitiesList()->getElement(i)->getSensorID() == 0) {
			return true;
		}
		if (id == msg.getBody()->getRangeSensorCapabilitiesList()->getElement(i)->getSensorID()) {
			return true;
		}
	}
	return false;
}

bool p_requested_configuration(QueryRangeSensorConfiguration msg, int id)
{
	if (msg.getBody()->getRangeSensorConfigurationList()->getNumberOfElements() == 0) {
		return true;
	}
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorConfigurationList()->getNumberOfElements(); i++) {
		if (msg.getBody()->getRangeSensorConfigurationList()->getElement(i)->getSensorID() == 0) {
			return true;
		}
		if (id == msg.getBody()->getRangeSensorConfigurationList()->getElement(i)->getSensorID()) {
			return true;
		}
	}
	return false;
}

bool p_requested_sensor_data(QueryRangeSensorData msg, int id)
{
	if (msg.getBody()->getQueryRangeSensorDataList()->getNumberOfElements() == 0) {
		return true;
	}
	for (unsigned int i = 0; i < msg.getBody()->getQueryRangeSensorDataList()->getNumberOfElements(); i++) {
		if (msg.getBody()->getQueryRangeSensorDataList()->getElement(i)->getSensorID() == 0) {
			return true;
		}
		if (id == msg.getBody()->getQueryRangeSensorDataList()->getElement(i)->getSensorID()) {
			return true;
		}
	}
	return false;
}

bool p_requested_geometric_properties_data(QuerySensorGeometricProperties msg, int id)
{
	if (msg.getBody()->getSensorIdList()->getNumberOfElements() == 0) {
		return true;
	}
	for (unsigned int i = 0; i < msg.getBody()->getSensorIdList()->getNumberOfElements(); i++) {
		if (msg.getBody()->getSensorIdList()->getElement(i)->getSensorID() == 0) {
			return true;
		}
		if (id == msg.getBody()->getSensorIdList()->getElement(i)->getSensorID()) {
			return true;
		}
	}
	return false;
}

void RangeSensor_ReceiveFSM::sendConfirmSensorConfigurationAction(SetRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	ROS_WARN("sendConfirmSensorConfigurationAction not implemented yet");
}

void RangeSensor_ReceiveFSM::sendReportRangeSensorCapabilitiesAction(QueryRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("RangeSensor", "sendReportRangeSensorCapabilities to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportRangeSensorCapabilities response;
	for (unsigned int idx_sensor = 0; idx_sensor < this->p_sensors.size(); idx_sensor++) {
		if (p_requested_capability(msg, p_sensors[idx_sensor]->id)) {
			response.getBody()->getRangeSensorCapabilitiesList()->addElement(*p_sensors[idx_sensor]->capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0));
		}
	}
	this->sendJausMessage(response, sender);
}

void RangeSensor_ReceiveFSM::sendReportRangeSensorCompressedDataAction(QueryRangeSensorCompressedData msg, std::string arg0, Receive::Body::ReceiveRec transportData)
{
	ROS_WARN("sendReportRangeSensorCompressedDataAction not implemented yet");
}

void RangeSensor_ReceiveFSM::sendReportRangeSensorConfigurationAction(QueryRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("RangeSensor", "sendReportRangeSensorConfiguration to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportRangeSensorConfiguration response;
	for (unsigned int idx_sensor = 0; idx_sensor < this->p_sensors.size(); idx_sensor++) {
		if (p_requested_configuration(msg, p_sensors[idx_sensor]->id)) {
			response.getBody()->getRangeSensorConfigurationList()->addElement(*p_sensors[idx_sensor]->configuration.getBody()->getRangeSensorConfigurationList()->getElement(0));
		}
	}
	this->sendJausMessage(response, sender);
}

void RangeSensor_ReceiveFSM::sendReportRangeSensorDataAction(QueryRangeSensorData msg, std::string arg0, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("RangeSensor", "sendReportRangeSensorConfiguration to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportRangeSensorData response;
	for (unsigned int idx_sensor = 0; idx_sensor < this->p_sensors.size(); idx_sensor++) {
		if (p_requested_sensor_data(msg, p_sensors[idx_sensor]->id)) {
			response.getBody()->getRangeSensorDataList()->addElement(*p_sensors[idx_sensor]->sensor_data.getBody()->getRangeSensorDataList()->getElement(0));
		}
	}
	this->sendJausMessage(response, sender);
}

void RangeSensor_ReceiveFSM::sendReportSensorGeometricPropertiesAction(QuerySensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("RangeSensor", "sendReportSensorGeometricProperties to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportSensorGeometricProperties response;
	for (unsigned int idx_sensor = 0; idx_sensor < this->p_sensors.size(); idx_sensor++) {
		if (p_requested_geometric_properties_data(msg, p_sensors[idx_sensor]->id)) {
			response.getBody()->getGeometricPropertiesList()->addElement(*p_sensors[idx_sensor]->geometric.getBody()->getGeometricPropertiesList()->getElement(0));
		}
	}
	this->sendJausMessage(response, sender);
}

void RangeSensor_ReceiveFSM::updateRangeSensorConfigurationAction()
{
	ROS_WARN("updateRangeSensorConfigurationAction not implemented yet");
}

bool RangeSensor_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}
bool RangeSensor_ReceiveFSM::isCoordinateTranformSupported()
{
	/// Insert User Code HERE
	return false;
}

};
