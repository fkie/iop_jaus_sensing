

#ifndef STILLIMAGE_RECEIVEFSM_H
#define STILLIMAGE_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_StillImage/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_StillImage/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h"


#include "StillImage_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>


namespace urn_jaus_jss_environmentSensing_StillImage
{

class DllExport StillImage_ReceiveFSM : public JTS::StateMachine
{
public:
	StillImage_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~StillImage_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void sendConfirmSensorConfigurationAction(SetStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportStillImageDataAction(QueryStillImageData msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportStillImageDataInNativeSystemAction(QueryStillImageData msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportStillImageSensorCapabilitiesAction(QueryStillImageSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportStillImageSensorConfigurationAction(QueryStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void updateStillImageSensorConfigurationAction(SetStillImageSensorConfiguration msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isCoordinateTransformSupported(QueryStillImageData msg);



	StillImage_ReceiveFSMContext *context;

protected:

	class ImageEndpoint {
	public:
		ImageEndpoint(std::shared_ptr<iop::Component> cmp, int id, std::string topic, StillImage_ReceiveFSM& parent);
		~ImageEndpoint();
		ReportStillImageData report;
		bool is_report_valid;
	private:
		std::shared_ptr<iop::Component> cmp;
		StillImage_ReceiveFSM* parent;
		int id;
		std::string topic;
		rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber;
		void ros_compressed_image_handler(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
	};

	/// References to parent FSMs
	urn_jaus_jss_environmentSensing_VisualSensor::VisualSensor_ReceiveFSM* pVisualSensor_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;
	std::map<int, std::shared_ptr<ImageEndpoint> > p_sensors;
	ReportStillImageSensorCapabilities p_report_capabilities;
	ReportStillImageSensorConfiguration p_report_configuration;
	// std::map<int, ReportStillImageData> p_report_image_data_map;

	// std::map<int, std::string> p_sensors;
	// std::map<std::string, int> p_sensor_ids;
	// std::map<int, rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr > p_subscriber;
	// DeVivo::Junior::JrMutex mutex;

	// void ros_compressed_image_handler(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
	unsigned short get_image_format(std::string format);
	bool is_requested(unsigned short sensor_id, QueryStillImageData &msg);
};

}

#endif // STILLIMAGE_RECEIVEFSM_H
