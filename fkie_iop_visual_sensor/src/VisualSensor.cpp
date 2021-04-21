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

#include <fkie_iop_builder/util.h>
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/string.hpp>
#include <fkie_iop_visual_sensor/VisualSensor.h>
#include <limits>
#include <string>


using namespace iop;
using namespace urn_jaus_jss_environmentSensing_VisualSensor;

VisualSensor::VisualSensor(jUnsignedShortInteger id, std::string name)
{
	p_id = id;
	p_name = name;
	p_switchable = false;
	p_switch_state = false;
	p_zoomable = false;
	p_zoom_level = 1;
	p_joint_nr = std::numeric_limits<uint8_t>::max();
	p_pose_valid = false;
	p_fov_horizontal = std::numeric_limits<float>::max();
	p_fov_vertical = std::numeric_limits<float>::max();
}

VisualSensor::VisualSensor(std::shared_ptr<iop::Component> cmp, jUnsignedShortInteger id, std::map<std::string, std::string>& params)
{
	p_id = id;
	p_name = "";
	p_switchable = false;
	p_switch_state = false;
	p_zoomable = false;
	p_zoom_level = 0;
	p_joint_nr = std::numeric_limits<uint8_t>::max();
	p_pose_valid = false;
	p_fov_horizontal = std::numeric_limits<float>::max();
	p_fov_vertical = std::numeric_limits<float>::max();
	init(cmp, id, params);
}

VisualSensor::~VisualSensor()
{
}

bool VisualSensor::operator==(VisualSensor &value)
{
	return p_id == value.p_id;
}

bool VisualSensor::operator!=(VisualSensor &value)
{
	return !(*this == value);
}

void VisualSensor::init(std::shared_ptr<iop::Component> cmp, jUnsignedShortInteger id, std::map<std::string, std::string>& params)
{
	p_id = id;
	auto logger = cmp->get_logger().get_child("VisualSensor");
	std::stringstream ss;
	ss << (int)id;
	std::string idstr("sensor_");
	idstr += ss.str();
	// parse the parameter
	iop::Config cmpcfg(cmp, "VisualSensor");
	std::map<std::string, std::string>::iterator pit;
	for (pit = params.begin(); pit != params.end(); pit++) {
		if (pit->first.compare("name") == 0) {
			p_name = static_cast<std::string>(pit->second);
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] name: %s", id, p_name.c_str());
		} else if (pit->first.compare("zoomable") == 0) {
			p_zoomable = stob(pit->second);
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] zoomable: %d", id, p_zoomable);
			if (p_zoomable) {
				p_pub_zoom_level = cmpcfg.create_publisher<std_msgs::msg::Float32>(idstr + "/cmd_zoom_level", 5);
				p_sub_zoom_level = cmpcfg.create_subscription<std_msgs::msg::Float32>(idstr + "/zoom_level", 5, std::bind(&VisualSensor::p_ros_zoom_level, this, std::placeholders::_1));
			}
		} else if (pit->first.compare("switchable") == 0) {
			p_switchable = stob(pit->second);
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] switchable: %d", id, p_switchable);
			if (p_switchable) {
				p_pub_state = cmpcfg.create_publisher<std_msgs::msg::Bool>(idstr + "/cmd_pwr_state", 5);
				p_sub_state = cmpcfg.create_subscription<std_msgs::msg::Bool>(idstr + "/pwr_state", 5, std::bind(&VisualSensor::p_ros_state, this, std::placeholders::_1));
			}
		} else if (pit->first.compare("fov_horizontal") == 0) {
			p_fov_horizontal = std::stof(pit->second);
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] fov_horizontal: %.2f", id, p_fov_horizontal);
			p_sub_fov_horizontal = cmpcfg.create_subscription<std_msgs::msg::Float32>(idstr + "/fov_horizontal", 5, std::bind(&VisualSensor::p_ros_fov_horizontal, this, std::placeholders::_1));
		} else if (pit->first.compare("fov_vertical") == 0) {
			p_fov_vertical = std::stof(pit->second);
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] fov_vertical: %.2f", id, p_fov_vertical);
			p_sub_fov_vertical = cmpcfg.create_subscription<std_msgs::msg::Float32>(idstr + "/fov_vertical", 5, std::bind(&VisualSensor::p_ros_fov_vertical, this, std::placeholders::_1));
		} else if (pit->first.compare("manipulator") == 0) {
			int p1, p2, p3;
			int scan_result = std::sscanf(pit->second.c_str(), "%d.%d.%d", &p1, &p2, &p3);
			if (scan_result == 3) {
				p_manipulator = JausAddress(p1, p2, p3);
			} else {
				RCLCPP_WARN(logger, "invalid format in manipulator[str]: %s, should be subsystem.node.component", pit->second.c_str());
			}
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] manipulator: %s", id, p_manipulator.str().c_str());
		} else if (pit->first.compare("manipulator_joint") == 0) {
			p_joint_nr = std::stoi(pit->second);
			RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] manipulator_joint: %d", id, p_joint_nr);
		} else if (pit->first.compare("pose") == 0) {
			std::string posestr = static_cast<std::string>(pit->second);
			float p1, p2, p3, q1, q2, q3, q4;
			int scan_result = std::sscanf(pit->second.c_str(), "%f %f %f %f %f %f %f", &p1, &p2, &p3, &q1, &q2, &q3, &q4);
			if (scan_result == 7) {
				p_pose.position.x = p1;
				p_pose.position.y = p2;
				p_pose.position.z = p3;
				p_pose.orientation.x = q1;
				p_pose.orientation.y = q2;
				p_pose.orientation.z = q3;
				p_pose.orientation.w = q4;
				p_pose_valid = true;
				RCLCPP_INFO(logger, "VisualSensor", "[visual sensor %d] pose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", id, p1, p2, p3, q1, q2, q3, q4);
			} else {
				RCLCPP_WARN(logger, "invalid format in pose[str]: %s, should be x y z q1 q2 q3 q4", pit->second.c_str());
			}
		}
	}
}

void VisualSensor::set_zoom_level(jUnsignedShortInteger level)
{
	if (is_zoomable()) {
		auto msg = std_msgs::msg::Float32();
		msg.data = (float)level;
		p_pub_zoom_level->publish(msg);
	}
}

void VisualSensor::set_state(jUnsignedByte state)
{
	if (is_switchable()) {
		p_switch_state = true;
		if (state == 2) {
			p_switch_state = false;
		}
		auto msg = std_msgs::msg::Bool();
		msg.data = p_switch_state;
		p_pub_state->publish(msg);
	}
}

CapabilityRec VisualSensor::get_capability()
{
	CapabilityRec result;
	result.setSensorID(p_id);
	result.setSensorName(p_name);
	ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec::ZoomModes zm;
	if (is_zoomable()) {
		zm.setMixed(1);
	} else {
		zm.setNone(1);
	}
	result.setZoomModes(zm);
	ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec::SupportedStates ss;
	ss.setActive(1);
	if (is_switchable()) {
		ss.setOff(1);
	}
	result.setSupportedStates(ss);
	return result;
}

ConfigurationRec VisualSensor::get_configuration()
{
	ConfigurationRec result;
	result.setSensorID(p_id);
	if (p_fov_horizontal != std::numeric_limits<float>::max()) {
		result.setHorizontalFieldOfView(p_fov_horizontal);
	}
	if (p_fov_vertical != std::numeric_limits<float>::max()) {
		result.setVerticalFieldOfView(p_fov_vertical);
	}
	result.setZoomLevel(p_zoom_level + 1.0);  // HACK: because the JAUS message transports a wrong value
	if (p_switch_state) {
		result.setSensorState(0);
	} else {
		result.setSensorState(2);
	}
	if (is_zoomable()) {
		result.setZoomMode(0);
	}
	return result;
}

GeometricSeq VisualSensor::get_geometric()
{
	GeometricSeq result;
	result.getSensorIdRec()->setSensorID(p_id);
	if (is_manipulator_valid() && is_joint_valid() && is_pose_valid()) {
		GeometricSeq::GeometricPropertiesVariant::ManipulatorGeometricPropertiesRec manip;
		manip.setSubsystemID(p_manipulator.getSubsystemID());
		manip.setNodeID(p_manipulator.getNodeID());
		manip.setComponentID(p_manipulator.getComponentID());
		manip.setJointNumber(p_joint_nr);
		manip.getSensorPosition()->setPositionVectorElement(0, p_pose.position.x);
		manip.getSensorPosition()->setPositionVectorElement(1, p_pose.position.y);
		manip.getSensorPosition()->setPositionVectorElement(2, p_pose.position.z);
		manip.getUnitQuaternion()->setUnitQuaternionElement(0, p_pose.orientation.x);
		manip.getUnitQuaternion()->setUnitQuaternionElement(1, p_pose.orientation.y);
		manip.getUnitQuaternion()->setUnitQuaternionElement(2, p_pose.orientation.z);
		manip.getUnitQuaternion()->setUnitQuaternionElement(3, p_pose.orientation.w);
		result.getGeometricPropertiesVariant()->setManipulatorGeometricPropertiesRec(manip);
	} else if (is_pose_valid()) {
		GeometricSeq::GeometricPropertiesVariant::StaticGeometricPropertiesRec pose;
		pose.getSensorPosition()->setPositionVectorElement(0, p_pose.position.x);
		pose.getSensorPosition()->setPositionVectorElement(1, p_pose.position.y);
		pose.getSensorPosition()->setPositionVectorElement(2, p_pose.position.z);
		pose.getUnitQuaternion()->setUnitQuaternionElement(0, p_pose.orientation.x);
		pose.getUnitQuaternion()->setUnitQuaternionElement(1, p_pose.orientation.y);
		pose.getUnitQuaternion()->setUnitQuaternionElement(2, p_pose.orientation.z);
		pose.getUnitQuaternion()->setUnitQuaternionElement(3, p_pose.orientation.w);
		result.getGeometricPropertiesVariant()->setStaticGeometricPropertiesRec(pose);
	} else {
		GeometricSeq::GeometricPropertiesVariant::NoGeometricPropertiesVariant nogeo;
		result.getGeometricPropertiesVariant()->setNoGeometricPropertiesVariant(nogeo);
	}
	return result;
}

void VisualSensor::apply_cfg(SetConfigurationRec* cfg)
{
	if (cfg->isSensorStateValid() && is_switchable()) {
		set_state(cfg->getSensorState());
	}
	if (cfg->isZoomLevelValid() && is_zoomable()) {
		set_zoom_level(pround(cfg->getZoomLevel()));
	}
}

void VisualSensor::p_ros_state(const std_msgs::msg::Bool::SharedPtr state)
{
	p_switch_state = state ->data;
	if (p_state_callback) {
		p_state_callback(p_id);
	}
}


void VisualSensor::p_ros_zoom_level(const std_msgs::msg::Float32::SharedPtr msg)
{
	p_zoom_level = msg->data;
	if (p_state_callback) {
		p_state_callback(p_id);
	}
}

void VisualSensor::p_ros_fov_horizontal(const std_msgs::msg::Float32::SharedPtr msg)
{
	p_fov_horizontal = msg->data;
	if (p_state_callback) {
		p_state_callback(p_id);
	}
}

void VisualSensor::p_ros_fov_vertical(const std_msgs::msg::Float32::SharedPtr msg)
{
	p_fov_vertical = msg->data;
	if (p_state_callback) {
		p_state_callback(p_id);
	}
}
