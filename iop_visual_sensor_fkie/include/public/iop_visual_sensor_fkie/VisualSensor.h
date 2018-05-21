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


#ifndef IOPVISUALSENSOR_H
#define IOPVISUALSENSOR_H

#include "Transport/JausTransport.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/InternalEvents/InternalEventsSet.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <string>

typedef urn_jaus_jss_environmentSensing_VisualSensor::ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec CapabilityRec;
typedef urn_jaus_jss_environmentSensing_VisualSensor::ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList::VisualSensorConfigurationRec ConfigurationRec;
typedef urn_jaus_jss_environmentSensing_VisualSensor::ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence GeometricSeq;
typedef urn_jaus_jss_environmentSensing_VisualSensor::SetVisualSensorConfiguration::Body::VisualSensorConfigurationSequence::VisualSensorConfigurationList::VisualSensorConfigurationRec SetConfigurationRec;

namespace iop
{

class VisualSensor {

public:
	VisualSensor(jUnsignedShortInteger id=0, std::string name="");
	VisualSensor(jUnsignedShortInteger id, XmlRpc::XmlRpcValue cfg);
	~VisualSensor();
	bool operator==(VisualSensor &value);
	bool operator!=(VisualSensor &value);
	void init(jUnsignedShortInteger id, XmlRpc::XmlRpcValue cfg);

	jUnsignedByte get_id() { return p_id; }
	std::string get_name() { return p_name; }
	jUnsignedShortInteger get_zoom_level() { return p_zoom_level; }
	bool get_switch_state() { return p_switch_state; } // 0: Active, 1: Standby, 2: Off
	JausAddress get_manipulator() { return p_manipulator; }
	jUnsignedByte get_joint_nr() { return p_joint_nr; }
	geometry_msgs::Pose get_pose() { return p_pose; }

	void set_zoom_level(jUnsignedShortInteger level);
	void set_state(jUnsignedByte state);

	/** Returns true if it was initialized and has valid id */
	bool is_valid() { return p_id != 0 && p_id != 65535; }
	bool is_zoomable() { return p_zoomable; }
	bool is_switchable() { return p_switchable; }
	bool is_manipulator_valid() { return p_manipulator.get() != 0; }
	bool is_joint_valid() { return p_joint_nr != std::numeric_limits<uint8_t>::max(); }
	bool is_pose_valid() { return p_pose_valid; }

	CapabilityRec get_capability();
	ConfigurationRec get_configuration();
	GeometricSeq get_geometric();
	void apply_cfg(SetConfigurationRec* cfg);

	template<class T>
	void set_state_callback(void(T::*handler)(jUnsignedShortInteger id), T*obj) {
		p_state_callback = boost::bind(handler, obj, _1);
	}

protected:
	boost::function<void (jUnsignedShortInteger id)> p_state_callback;
	jUnsignedShortInteger p_id;
	std::string p_name;
	bool p_switchable;  // 0: Active, 1: Standby, 2: Off
	bool p_switch_state;
	bool p_zoomable;  // 0: MixedZoom, 1: AnalogZoomOnly, 2: DigitalZoomOnly, 3: NoZoom
	jUnsignedShortInteger p_zoom_level;
	JausAddress p_manipulator;
	jUnsignedByte p_joint_nr;
	geometry_msgs::Pose p_pose;
	bool p_pose_valid;
	double p_fov_horizontal;
	double p_fov_vertical;
	ros::Publisher p_pub_state;
	ros::Subscriber p_sub_state;
	ros::Publisher p_pub_zoom_level;
	ros::Subscriber p_sub_zoom_level;
	ros::Subscriber p_sub_fov_horizontal;
	ros::Subscriber p_sub_fov_vertical;

private:
	VisualSensor(VisualSensor const& from);
	const VisualSensor& operator=(const VisualSensor& from);

	void p_ros_state(const std_msgs::Bool::ConstPtr& state);
	void p_ros_zoom_level(const std_msgs::Float32::ConstPtr& state);
	void p_ros_fov_horizontal(const std_msgs::Float32::ConstPtr& state);
	void p_ros_fov_vertical(const std_msgs::Float32::ConstPtr& state);
};

};

#endif
