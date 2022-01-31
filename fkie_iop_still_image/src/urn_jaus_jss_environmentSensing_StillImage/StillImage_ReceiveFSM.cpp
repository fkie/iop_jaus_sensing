

#include "urn_jaus_jss_environmentSensing_StillImage/StillImage_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/string.hpp>




using namespace JTS;
using namespace urn_jaus_jss_core_Events;

namespace urn_jaus_jss_environmentSensing_StillImage
{



StillImage_ReceiveFSM::StillImage_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("StillImage"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StillImage_ReceiveFSMContext(*this);

	this->pVisualSensor_ReceiveFSM = pVisualSensor_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
}



StillImage_ReceiveFSM::~StillImage_ReceiveFSM()
{
	delete context;
}

void StillImage_ReceiveFSM::setupNotifications()
{
	pVisualSensor_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_StillImage_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	pVisualSensor_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_StillImage_ReceiveFSM_Receiving_Ready_Controlled", "VisualSensor_ReceiveFSM");
	pVisualSensor_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_StillImage_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	pVisualSensor_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_StillImage_ReceiveFSM_Receiving_Ready_NotControlled", "VisualSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_NotControlled", "StillImage_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready_Controlled", "StillImage_ReceiveFSM");
	registerNotification("Receiving_Ready", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving_Ready", "StillImage_ReceiveFSM");
	registerNotification("Receiving", pVisualSensor_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensor_ReceiveFSM_Receiving", "StillImage_ReceiveFSM");

}


void StillImage_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "StillImage");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryStillImageData::ID, true);
	std::vector<std::string> sensors;
	cfg.declare_param<std::vector<std::string> >("image_topics", sensors, false,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"List of pairs of type {ID.ROS_TOPIC_NAME}. Example: - 4.map_image/compressed. The topics must have a type of sensor_msgs::CompressedImage.",
		"Default: []");
	cfg.param_vector<std::vector<std::string> >("image_topics", sensors, sensors);
	for (unsigned int i = 0; i < sensors.size(); i++) {
		auto entry = iop::split(iop::trim(sensors[i]), '.', 2);
		if (entry.size() == 2) {
			int ep_id = std::atoi(entry[0].c_str());
			std::string ep_topic = entry[1];
			if (ep_id == 0) {
				RCLCPP_WARN(logger, "sensor id 0 is reserved, please fix configuration!");
			} else {
				RCLCPP_INFO(logger, "Found image_topic <id: %d, topic: %s>", ep_id, ep_topic.c_str());
			}

			p_sensors[ep_id] = std::make_shared<StillImage_ReceiveFSM::ImageEndpoint>(cmp, ep_id, ep_topic, *this);
			ReportStillImageSensorCapabilities::Body::StillImageSensorList::StillImageSensorCapabilitiesRec scap_rec;
			scap_rec.setSensorID(ep_id);
			p_report_capabilities.getBody()->getStillImageSensorList()->addElement(scap_rec);
			ReportStillImageSensorConfiguration::Body::StillImageSensorConfigurationList::StillImageSensorConfigurationRec sconf_rec;
			sconf_rec.setSensorID(ep_id);
			p_report_configuration.getBody()->getStillImageSensorConfigurationList()->addElement(sconf_rec);

		} else {
			RCLCPP_WARN(logger, "skipped image_topic entry '%s' because of invalid format, expected list of: ID.ROS_TOPIC_NAME", sensors[i].c_str());
		}
	}
}

StillImage_ReceiveFSM::ImageEndpoint::ImageEndpoint(std::shared_ptr<iop::Component> cmp, int id, std::string topic, StillImage_ReceiveFSM& parent)
{
	this->id = id;
	this->topic = topic;
	this->cmp = cmp;
	this->parent = &parent;
	is_report_valid = false;
	iop::Config cfg(cmp, "ImageEndpoint");
	subscriber = cfg.create_subscription<sensor_msgs::msg::CompressedImage>(topic, 1, std::bind(&StillImage_ReceiveFSM::ImageEndpoint::ros_compressed_image_handler, this, std::placeholders::_1));
}

StillImage_ReceiveFSM::ImageEndpoint::~ImageEndpoint()
{

}

void StillImage_ReceiveFSM::ImageEndpoint::ros_compressed_image_handler(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
	/*
	*
	*/
	lock_type lock(parent->p_mutex);
	report = ReportStillImageData();
	ReportStillImageData::Body::StillImageDataList::StillImageDataRec rep_rec;
	rep_rec.setSensorID(id);
	ReportStillImageData::Body::StillImageDataList::StillImageDataRec::ImageFrame img_frame;
	unsigned short format_id = parent->get_image_format(msg->format);
	// TODO update format in capabilities and configuration reports
	img_frame.set(format_id, msg->data.size(), (unsigned char *)&msg->data[0]);
	rep_rec.setImageFrame(img_frame);
	report.getBody()->getStillImageDataList()->addElement(rep_rec);
	// apply query filter
	std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage> queries;
	parent->pEvents_ReceiveFSM->get_event_handler().get_queries(QueryStillImageData::ID, queries);
	std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage>::iterator it;
	for (it = queries.begin(); it != queries.end(); ++it) {
		QueryStillImageData qr;
		qr.decode(it->second.getData());
		unsigned int qr_len = qr.getBody()->getQueryStillImageDataList()->getNumberOfElements();
		for (unsigned int q = 0; q < qr_len; q++) {
			QueryStillImageData::Body::QueryStillImageDataList::QueryStillImageDataRec *qr_rec;
			qr_rec = qr.getBody()->getQueryStillImageDataList()->getElement(q);
			if (qr_rec->getSensorID() == id || qr_rec->getSensorID() == 0 || qr_rec->getSensorID() == 65535) {
				parent->pEvents_ReceiveFSM->get_event_handler().send_report(it->first, report, id);
			}
		}
	}
	is_report_valid = true;
}

void StillImage_ReceiveFSM::sendConfirmSensorConfigurationAction(SetStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "sendConfirmSensorConfigurationAction not implemented!");
}

void StillImage_ReceiveFSM::sendReportStillImageDataAction(QueryStillImageData msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	std::map<int, std::shared_ptr<ImageEndpoint> >::iterator it;
	for (it = p_sensors.begin(); it != p_sensors.end(); ++it) {
		if (is_requested(it->first, msg) && it->second->is_report_valid) {
			RCLCPP_DEBUG(logger, "sendReportStillImageData for sensor %d to %s", it->first, sender.str().c_str());
			sendJausMessage(it->second->report, sender);
		}
	}
}

void StillImage_ReceiveFSM::sendReportStillImageDataInNativeSystemAction(QueryStillImageData msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	std::map<int, std::shared_ptr<ImageEndpoint> >::iterator it;
	for (it = p_sensors.begin(); it != p_sensors.end(); ++it) {
		if (is_requested(it->first, msg) && it->second->is_report_valid) {
			RCLCPP_DEBUG(logger, "sendReportStillImageDataInNative for sensor %d to %s", it->first, sender.str().c_str());
			sendJausMessage(it->second->report, sender);
		}
	}
}

void StillImage_ReceiveFSM::sendReportStillImageSensorCapabilitiesAction(QueryStillImageSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	RCLCPP_DEBUG(logger, "sendReportStillImageSensorCapabilitiesAction to %s", sender.str().c_str());
	sendJausMessage(p_report_capabilities, sender);
}

void StillImage_ReceiveFSM::sendReportStillImageSensorConfigurationAction(QueryStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	RCLCPP_DEBUG(logger, "sendReportStillImageSensorConfigurationAction to %s", sender.str().c_str());
	sendJausMessage(p_report_configuration, sender);
}

void StillImage_ReceiveFSM::updateStillImageSensorConfigurationAction(SetStillImageSensorConfiguration msg)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "updateStillImageSensorConfigurationAction not implemented!");
}



bool StillImage_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool StillImage_ReceiveFSM::isCoordinateTransformSupported(QueryStillImageData msg)
{
	/// Insert User Code HERE
	return false;
}

bool StillImage_ReceiveFSM::is_requested(unsigned short sensor_id, QueryStillImageData &msg)
{
	for (unsigned int i = 0; i < msg.getBody()->getQueryStillImageDataList()->getNumberOfElements(); i++) {
		unsigned short req_id = msg.getBody()->getQueryStillImageDataList()->getElement(i)->getSensorID();
		if (req_id == sensor_id) {
			return true;
		}
		if (req_id == 0) {
			return true;
		}
		if (req_id == 65535) {
			return true;
		}
	}
	return false;
}

unsigned short StillImage_ReceiveFSM::get_image_format(std::string format)
{
	/*
	0: JPEG
	1: GIF
	2: PNG
	3: BMP
	4: TIFF
	5: PPM
	6: PGM
	7: PNM
	8: NEF
	9: CR_2
	10: DNG
	*/
	if (format.find("jpeg") != std::string::npos or format.find("JPEG") != std::string::npos) {
		return 0;
	}
	if (format.find("gif") != std::string::npos or format.find("GIF") != std::string::npos) {
		return 1;
	}
	if (format.find("png") != std::string::npos or format.find("PNG") != std::string::npos) {
		return 2;
	}
	if (format.find("bmp") != std::string::npos or format.find("BMP") != std::string::npos) {
		return 3;
	}
	if (format.find("tiff") != std::string::npos or format.find("TIFF") != std::string::npos) {
		return 4;
	}
	if (format.find("PPM") != std::string::npos or format.find("PPM") != std::string::npos) {
		return 5;
	}
	if (format.find("pgm") != std::string::npos or format.find("PGM") != std::string::npos) {
		return 6;
	}
	if (format.find("pnm") != std::string::npos or format.find("PNM") != std::string::npos) {
		return 7;
	}
	RCLCPP_WARN(logger, "can't determine JAUS image format from: %s", format.c_str());
	return 2;
}


}
