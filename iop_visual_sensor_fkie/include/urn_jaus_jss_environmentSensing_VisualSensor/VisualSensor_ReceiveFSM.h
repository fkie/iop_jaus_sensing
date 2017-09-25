

#ifndef VISUALSENSOR_RECEIVEFSM_H
#define VISUALSENSOR_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"


#include "VisualSensor_ReceiveFSM_sm.h"

namespace urn_jaus_jss_environmentSensing_VisualSensor
{

class DllExport VisualSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	VisualSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~VisualSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void sendConfirmSensorConfigurationAction(SetVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportSensorGeometricPropertiesAction(QuerySensorGeometricProperties msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportVisualSensorCapabilitiesAction(QueryVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportVisualSensorConfigurationAction(QueryVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void updateVisualSensorConfigurationAction(SetVisualSensorConfiguration msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);


	VisualSensor_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	std::map<unsigned short, std::string> p_sensor_names;

};

};

#endif // VISUALSENSOR_RECEIVEFSM_H
