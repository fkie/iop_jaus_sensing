

#include "urn_jaus_jss_environmentSensing_StillImage/StillImage_ReceiveFSM.h"
#include <iop_component_fkie/iop_config.h>




using namespace JTS;
using namespace urn_jaus_jss_core_Events;

namespace urn_jaus_jss_environmentSensing_StillImage
{



StillImage_ReceiveFSM::StillImage_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StillImage_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pVisualSensor_ReceiveFSM = pVisualSensor_ReceiveFSM;
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
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryStillImageData::ID, true);
	iop::Config cfg("~StillImage");
	XmlRpc::XmlRpcValue sensors;
	cfg.param("image_sensors", sensors, sensors);
	if (!sensors.valid()) {
		ROS_ERROR_NAMED("StillImage", "wrong '~image_sensors' format, expected list of: ID: TOPIC.\n  ID: ressource id {0..254}");
		ROS_BREAK();
	}
	// parse the parameter into a map
	for(int i = 0; i < sensors.size(); i++) {
		if (sensors[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = sensors[i].begin(); iterator != sensors[i].end(); iterator++) {
				std::string vi_type = iterator->first;
				int ep_id = std::atoi(vi_type.c_str());
				if (ep_id == 0) {
					ROS_WARN_NAMED("StillImage", "sensor id 0 is reserved, please fix configuration!");
				}
				std::string vi_uri = iterator->second;
				p_sensors[ep_id] = vi_uri;
				p_subscriber[ep_id] = cfg.subscribe(vi_uri, 1, &StillImage_ReceiveFSM::ros_compressed_image_handler, this);;
				p_sensor_ids[p_subscriber.at(ep_id).getTopic()] = ep_id;
				ReportStillImageSensorCapabilities::Body::StillImageSensorList::StillImageSensorCapabilitiesRec scap_rec;
				scap_rec.setSensorID(ep_id);
				p_report_capabilities.getBody()->getStillImageSensorList()->addElement(scap_rec);
				ReportStillImageSensorConfiguration::Body::StillImageSensorConfigurationList::StillImageSensorConfigurationRec sconf_rec;
				sconf_rec.setSensorID(ep_id);
				p_report_configuration.getBody()->getStillImageSensorConfigurationList()->addElement(sconf_rec);
			}
		} else {
			ROS_ERROR_NAMED("StillImage", "wrong entry of '~sensors', expected: 'ID: ROS_TOPIC'.\n  ID: int value 0-254\n  ROS_TOPIC: string");
		}
	}

}

void StillImage_ReceiveFSM::sendConfirmSensorConfigurationAction(SetStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_WARN_NAMED("StillImage", "sendConfirmSensorConfigurationAction not implemented!");
}

void StillImage_ReceiveFSM::sendReportStillImageDataAction(QueryStillImageData msg, Receive::Body::ReceiveRec transportData)
{
	mutex.lock();
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	std::map<int, ReportStillImageData>::iterator it;
	for (it = p_report_image_data_map.begin(); it != p_report_image_data_map.end(); ++it) {
		if (is_requested(it->first, msg)) {
			ROS_DEBUG_NAMED("StillImage", "sendReportStillImageData for sensor %d to %s", it->first, sender.str().c_str());
			sendJausMessage(it->second, sender);
		}
	}
	mutex.unlock();
}

void StillImage_ReceiveFSM::sendReportStillImageDataInNativeSystemAction(QueryStillImageData msg, Receive::Body::ReceiveRec transportData)
{
	mutex.lock();
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	std::map<int, ReportStillImageData>::iterator it;
	for (it = p_report_image_data_map.begin(); it != p_report_image_data_map.end(); ++it) {
		if (is_requested(it->first, msg)) {
			ROS_DEBUG_NAMED("StillImage", "sendReportStillImageDataInNative for sensor %d to %s", it->first, sender.str().c_str());
			sendJausMessage(it->second, sender);
		}
	}
	mutex.unlock();
}

void StillImage_ReceiveFSM::sendReportStillImageSensorCapabilitiesAction(QueryStillImageSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	ROS_DEBUG_NAMED("StillImage", "sendReportStillImageSensorCapabilitiesAction to %s", sender.str().c_str());
	sendJausMessage(p_report_capabilities, sender);
}

void StillImage_ReceiveFSM::sendReportStillImageSensorConfigurationAction(QueryStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender(transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	ROS_DEBUG_NAMED("StillImage", "sendReportStillImageSensorConfigurationAction to %s", sender.str().c_str());
	sendJausMessage(p_report_configuration, sender);
}

void StillImage_ReceiveFSM::updateStillImageSensorConfigurationAction(SetStillImageSensorConfiguration msg)
{
	/// Insert User Code HERE
	ROS_WARN_NAMED("StillImage", "updateStillImageSensorConfigurationAction not implemented!");
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

void StillImage_ReceiveFSM::ros_compressed_image_handler(const ros::MessageEvent<const sensor_msgs::CompressedImage >& event)
{
	/*
	*
	*/
	const sensor_msgs::CompressedImage& msg = *event.getMessage();
	std::string topic_name = event.getConnectionHeader()["topic"];
	std::map<std::string, int>::iterator it = p_sensor_ids.find(topic_name);
	if (it != p_sensor_ids.end()) {
		int id = p_sensor_ids[topic_name];
		ROS_INFO_ONCE_NAMED("StillImage", "received CompressedImage from: %s", topic_name.c_str());
		mutex.lock();
		p_report_image_data_map[id] = ReportStillImageData();
		ReportStillImageData &report = p_report_image_data_map[id];
		ReportStillImageData::Body::StillImageDataList::StillImageDataRec rep_rec;
		rep_rec.setSensorID(id);
		ReportStillImageData::Body::StillImageDataList::StillImageDataRec::ImageFrame img_frame;
		unsigned short format_id = get_image_format(msg.format);
		// TODO update format in capabilities and configuration reports
		img_frame.set(format_id, msg.data.size(), (unsigned char *)&msg.data[0]);
		rep_rec.setImageFrame(img_frame);
		report.getBody()->getStillImageDataList()->addElement(rep_rec);
		// apply query filter
		std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage> queries;
		pEvents_ReceiveFSM->get_event_handler().get_queries(QueryStillImageData::ID, queries);
		std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage>::iterator it;
		for (it = queries.begin(); it != queries.end(); ++it) {
			QueryStillImageData qr;
			qr.decode(it->second.getData());
			unsigned int qr_len = qr.getBody()->getQueryStillImageDataList()->getNumberOfElements();
			for (unsigned int q = 0; q < qr_len; q++) {
				QueryStillImageData::Body::QueryStillImageDataList::QueryStillImageDataRec *qr_rec;
				qr_rec = qr.getBody()->getQueryStillImageDataList()->getElement(q);
				if (qr_rec->getSensorID() == id || qr_rec->getSensorID() == 0 || qr_rec->getSensorID() == 65535) {
					pEvents_ReceiveFSM->get_event_handler().send_report(it->first, report, id);
				}
			}
		}
		mutex.unlock();
	}
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
	ROS_WARN_NAMED("StillImage", "can't determine JAUS image format from: %s", format.c_str());
	return 2;
}


};
