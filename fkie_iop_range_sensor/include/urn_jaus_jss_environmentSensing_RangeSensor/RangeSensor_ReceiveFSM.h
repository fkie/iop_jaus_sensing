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

#include "JausUtils.h"
#include <string>
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_RangeSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_RangeSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"

#include "RangeSensor_ReceiveFSM_sm.h"
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>


namespace urn_jaus_jss_environmentSensing_RangeSensor
{

class DllExport RangeSensor_ReceiveFSM : public JTS::StateMachine
{
//friend RangeSensor;
public:
	RangeSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~RangeSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

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
		RangeSensor(std::shared_ptr<iop::Component> cmp, int id, std::string topic, std::string tf_frame_robot, RangeSensor_ReceiveFSM &parent);
		~RangeSensor();
		int id;
		bool initialized;
		std::string ros_topic;
		std::string tf_frame_robot;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ros_sub;
		ReportRangeSensorCapabilities capabilities;
		ReportSensorGeometricProperties geometric;
		ReportRangeSensorConfiguration configuration;
		ReportRangeSensorData sensor_data;
		void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
	private:
		RangeSensor_ReceiveFSM *parent;

	};
	/// References to parent FSMs
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	typedef std::shared_ptr<sensor_msgs::msg::LaserScan const> LaserScanConstPtr;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;
	std::vector<RangeSensor *> p_sensors;
	std::string p_tf_frame_robot;
	std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> p_tf_listener;

	void stop_subscriber();

};

}

#endif // RANGESENSOR_RECEIVEFSM_H
