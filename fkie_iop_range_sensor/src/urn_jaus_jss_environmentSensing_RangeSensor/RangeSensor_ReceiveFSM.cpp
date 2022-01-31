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
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/timestamp.hpp>

#include <iostream>
#include <string>
#include <libgen.h>
#include "JausUtils.h"

#include <geometry_msgs/msg/transform_stamped.hpp>


using namespace JTS;
using namespace urn_jaus_jss_core_Transport;
using namespace urn_jaus_jss_core_Events;
using namespace urn_jaus_jss_core_AccessControl;
using namespace urn_jaus_jss_environmentSensing_RangeSensor;



using namespace JTS;

namespace urn_jaus_jss_environmentSensing_RangeSensor
{



RangeSensor_ReceiveFSM::RangeSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("RangeSensor"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new RangeSensor_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
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
}


void RangeSensor_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "RangeSensor");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryRangeSensorData::ID);
	stop_subscriber();
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame used in ROS for local coordinates. This value is set in each command message.",
		"Default: 'base_link'");
	std::vector<std::string> sensors;
	cfg.declare_param<std::vector<std::string> >("range_sensors", sensors, false,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"A list with laser scan topics.",
		"Default: []");

	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	int sensor_id = 1;  // id 0 is reserved for all
	lock_type lock(p_mutex);
	p_tf_buffer = std::make_unique<tf2_ros::Buffer>(cmp->get_clock());
	p_tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);
	cfg.param_vector<std::vector<std::string> >("range_sensors", sensors, sensors);
	for(unsigned int i = 0; i < sensors.size(); i++) {
		std::string ros_topic = sensors[i];
		// resolve to node namespace
		RangeSensor *sensor = new RangeSensor(cmp, sensor_id, ros_topic, p_tf_frame_robot, *this);
		sensor_id++;
		p_sensors.push_back(sensor);
	}

	// } else {
	// 	RCLCPP_WARN(logger, "wrong ~sensors parameter type! It should be an array with format [ros_topic, ...]");
	// }
}

RangeSensor_ReceiveFSM::RangeSensor::RangeSensor(std::shared_ptr<iop::Component> cmp, int id, std::string topic, std::string tf_frame_robot, RangeSensor_ReceiveFSM &parent)
{
	this->id = id;
	this->initialized = false;
	this->tf_frame_robot = tf_frame_robot;
	this->parent = &parent;
	this->ros_topic = topic;
	iop::Config cfg(cmp, "RangeSensor_" + topic);
	this->ros_sub = cfg.create_subscription<sensor_msgs::msg::LaserScan>(topic, 1, std::bind(&RangeSensor_ReceiveFSM::RangeSensor::scan_callback, this, std::placeholders::_1));
	ReportRangeSensorCapabilities::Body::RangeSensorCapabilitiesList::RangeSensorCapabilitiesRec caprec;
	caprec.setSensorID(id);
	caprec.setSensorName(std::string(basename((char *)topic.c_str()))); // or this->ros_sub->get_topic_name() ?
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
	datavar.getRangeSensorDataErrorRec()->setErrorMessage("no data available");
	sensor_data.getBody()->getRangeSensorDataList()->addElement(datavar);
}

RangeSensor_ReceiveFSM::RangeSensor::~RangeSensor()
{
	//ros_sub.shutdown();
}

void RangeSensor_ReceiveFSM::RangeSensor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
	lock_type lock(this->parent->p_mutex);
	if (!this->initialized) {
		// try to get the robot coordinate system
		bool geo_init = false;
		if (!tf_frame_robot.empty()) {
			try {
  				auto ts = parent->p_tf_buffer->lookupTransform(tf_frame_robot, scan->header.frame_id, scan->header.stamp, rclcpp::Duration::from_seconds(0.3));
				ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence::GeometricPropertiesVariant::StaticGeometricPropertiesRec staticgeo;
				staticgeo.getSensorPosition()->setPositionVectorElement(0, ts.transform.translation.x);
				staticgeo.getSensorPosition()->setPositionVectorElement(1, ts.transform.translation.y);
				staticgeo.getSensorPosition()->setPositionVectorElement(2, ts.transform.translation.z);
				staticgeo.getUnitQuaternion()->setUnitQuaternionElement(0, ts.transform.rotation.x);
				staticgeo.getUnitQuaternion()->setUnitQuaternionElement(1, ts.transform.rotation.y);
				staticgeo.getUnitQuaternion()->setUnitQuaternionElement(2, ts.transform.rotation.z);
				staticgeo.getUnitQuaternion()->setUnitQuaternionElement(3, ts.transform.rotation.w);
				this->geometric.getBody()->getGeometricPropertiesList()->getElement(0)->getGeometricPropertiesVariant()->setStaticGeometricPropertiesRec(staticgeo);
				this->geometric.getBody()->getGeometricPropertiesList()->getElement(0)->getGeometricPropertiesVariant()->setFieldValue(1);
				geo_init = true;
				RCLCPP_DEBUG(parent->logger, "initialized geometrics for range sensor %s", this->ros_sub->get_topic_name());
  			} catch (tf2::TransformException &ex) {
				rclcpp::Clock steady_clock(RCL_STEADY_TIME);
				RCLCPP_WARN_THROTTLE(parent->logger, steady_clock, 10000, "Could not lookup transform from %s to %s: %s", tf_frame_robot.c_str(), scan->header.frame_id.c_str(), ex.what());
			}
		}
		capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMinimumHorizontalFieldOfViewStartAngle(scan->angle_min);
		capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMaximumHorizontalFieldOfViewStopAngle(scan->angle_max);
		capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMinimumRange(scan->range_min);
		capabilities.getBody()->getRangeSensorCapabilitiesList()->getElement(0)->setMaximumRange(scan->range_max);
		configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setSensorState(0);
		configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setHorizontalFieldOfViewStartAngle(scan->angle_min);
		configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setHorizontalFieldOfViewStopAngle(scan->angle_max);
		configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setMinimumRange(scan->range_min);
		configuration.getBody()->getRangeSensorConfigurationList()->getElement(0)->setMaximumRange(scan->range_max);
		if (geo_init) {
			this->initialized = true;
		}
	}
	if (sensor_data.getBody()->getRangeSensorDataList()->getNumberOfElements() > 0)
	{
		sensor_data.getBody()->getRangeSensorDataList()->deleteLastElement();
	}
	ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant sdatavar;
	// set timestamp
	ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataRec::TimeStamp ts;
	// current date/time based on current system
	iop::Timestamp stamp(scan->header.stamp);
	ts.setDay(stamp.days);
	ts.setHour(stamp.hours);
	ts.setMinutes(stamp.minutes);
	ts.setSeconds(stamp.seconds);
	ts.setMilliseconds(stamp.milliseconds);
	sdatavar.getRangeSensorDataSeq()->getRangeSensorDataRec()->setSensorID(id);
	sdatavar.getRangeSensorDataSeq()->getRangeSensorDataRec()->setTimeStamp(ts);
	sdatavar.getRangeSensorDataSeq()->getRangeSensorDataRec()->setReportCoordinateSystem(1);
	// set points
	ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataPointList::RangeSensorDataPointRec point;
	for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
		point.setBearing(scan->angle_min + (i * scan->angle_increment));
		if (std::isnan(scan->ranges[i])) {
			point.setRangeValidity(0);
		} else {
			int res = point.setRange(scan->ranges[i]);
			if (res == 0) {
				point.setRangeValidity(1);
			} else {
				point.setRangeValidity(0);
			}
		}
		point.setInclination(0.0);
		sdatavar.getRangeSensorDataSeq()->getRangeSensorDataPointList()->addElement(point);
	}
	sdatavar.setFieldValue(1);
	sensor_data.getBody()->getRangeSensorDataList()->addElement(sdatavar);
	// apply query filter
	std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage> queries;
	parent->pEvents_ReceiveFSM->get_event_handler().get_queries(QueryRangeSensorData::ID, queries);
	std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage>::iterator it;
	for (it = queries.begin(); it != queries.end(); ++it) {
		QueryRangeSensorData qr;
		qr.decode(it->second.getData());
		unsigned int qr_len = qr.getBody()->getQueryRangeSensorDataList()->getNumberOfElements();
		for (unsigned int q = 0; q < qr_len; q++) {
			QueryRangeSensorData::Body::QueryRangeSensorDataList::QueryRangeSensorDataRec *qr_rec;
			qr_rec = qr.getBody()->getQueryRangeSensorDataList()->getElement(q);
			if (qr_rec->getSensorID() == id || qr_rec->getSensorID() == 0 || qr_rec->getSensorID() == 65535) {
				ReportRangeSensorData sensor_data_report;
				sensor_data_report.getBody()->getRangeSensorDataList()->addElement(sdatavar);
				parent->pEvents_ReceiveFSM->get_event_handler().send_report(it->first, sensor_data_report, id);
			}
		}
	}
//		pEvents_ReceiveFSM->get_event_handler().set_report(QueryRangeSensorData::ID, &sensor->sensor_data);
}


void RangeSensor_ReceiveFSM::stop_subscriber()
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_sensors.size(); i++) {
		delete p_sensors[i];
	}
	p_sensors.clear();
}

bool p_requested_capability(QueryRangeSensorCapabilities msg, int id)
{
	if (msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements() == 0) {
		return true;
	}
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements(); i++) {
		unsigned short requsted_id = msg.getBody()->getRangeSensorCapabilitiesList()->getElement(i)->getSensorID();
		if (requsted_id == 0 || requsted_id == 65535) {
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
		unsigned short requsted_id = msg.getBody()->getRangeSensorConfigurationList()->getElement(i)->getSensorID();
		if (requsted_id == 0 || requsted_id == 65535) {
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
		unsigned short requsted_id = msg.getBody()->getQueryRangeSensorDataList()->getElement(i)->getSensorID();
		if (requsted_id == 0 || requsted_id == 65535) {
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
		unsigned short requsted_id = msg.getBody()->getSensorIdList()->getElement(i)->getSensorID();
		if (requsted_id == 0 || requsted_id == 65535) {
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
	RCLCPP_WARN(logger, "sendConfirmSensorConfigurationAction not implemented yet");
}

void RangeSensor_ReceiveFSM::sendReportRangeSensorCapabilitiesAction(QueryRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportRangeSensorCapabilities to %s", sender.str().c_str());
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
	RCLCPP_WARN(logger, "sendReportRangeSensorCompressedDataAction not implemented yet");
}

void RangeSensor_ReceiveFSM::sendReportRangeSensorConfigurationAction(QueryRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportRangeSensorConfiguration to %s", sender.str().c_str());
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
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportRangeSensorData to %s", sender.str().c_str());
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
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportSensorGeometricProperties to %s", sender.str().c_str());
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
	RCLCPP_WARN(logger, "updateRangeSensorConfigurationAction not implemented yet");
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

}
