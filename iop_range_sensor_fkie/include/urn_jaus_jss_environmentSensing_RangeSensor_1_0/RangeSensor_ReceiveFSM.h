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


#ifndef RANGESENSOR_RECEIVEFSM_H
#define RANGESENSOR_RECEIVEFSM_H

#include <sensor_msgs/LaserScan.h>
#include "JausUtils.h"
#include <string>
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_RangeSensor_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_RangeSensor_1_0/InternalEvents/InternalEventsSet.h"
#include <tf/transform_listener.h>

typedef JTS::Receive_1_0 Receive;
typedef JTS::Send_1_0 Send;

#include "urn_jaus_jss_core_Transport_1_0/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events_1_0/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl_1_0/AccessControl_ReceiveFSM.h"

#include "ros/ros.h"
#include <boost/thread/recursive_mutex.hpp>

#include "RangeSensor_ReceiveFSM_sm.h"

namespace urn_jaus_jss_environmentSensing_RangeSensor_1_0
{

class DllExport RangeSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	RangeSensor_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~RangeSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void sendConfirmSensorConfigurationAction(SetRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportRangeSensorCapabilitiesAction(QueryRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportRangeSensorCompressedDataAction(QueryRangeSensorCompressedData msg, std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void sendReportRangeSensorConfigurationAction(QueryRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportRangeSensorDataAction(QueryRangeSensorData msg, std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void sendReportSensorGeometricPropertiesAction(QuerySensorGeometricProperties msg, Receive::Body::ReceiveRec transportData);
	virtual void updateRangeSensorConfigurationAction();


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isCoordinateTranformSupported();



	RangeSensor_ReceiveFSMContext *context;

protected:

	class RangeSensor {
	public:
		RangeSensor(int id, std::string topic, RangeSensor_ReceiveFSM *parent);
		~RangeSensor();
		int id;
		bool initialized;
		std::string ros_topic;
		ros::Subscriber ros_sub;
		ReportRangeSensorCapabilities capabilities;
		ReportSensorGeometricProperties geometric;
		ReportRangeSensorConfiguration configuration;
		ReportRangeSensorData sensor_data;
	};
    /// References to parent FSMs
	urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	boost::recursive_mutex p_mutex;
	std::vector<RangeSensor *> p_sensors;
	std::string p_tf_frame_robot;
	tf::TransformListener tfListener;

	void stop_subscriber();
	void scan_callback(const ros::MessageEvent<sensor_msgs::LaserScan const>& event);
};

};

#endif // RANGESENSOR_RECEIVEFSM_H
