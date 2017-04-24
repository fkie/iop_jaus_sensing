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


#ifndef DIGITALVIDEOPLUGIN_1_0_H
#define DIGITALVIDEOPLUGIN_1_0_H

#include "urn_jaus_jss_environmentSensing_DigitalVideo_1_0/DigitalVideoService.h"
#include "urn_jaus_jss_environmentSensing_VisualSensor_1_0/VisualSensorService.h"
#include "urn_jaus_jss_core_AccessControl_1_0/AccessControlService.h"
#include "urn_jaus_jss_core_Events_1_0/EventsService.h"
#include "urn_jaus_jss_core_Transport_1_0/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport DigitalVideoPlugin_1_0 : public PluginInterface
{
public:
	DigitalVideoPlugin_1_0();

	JTS::Service* get_iop_service();
	const std::type_info & get_iop_service_type();
	const std::type_info & get_base_service_type();
	void create_jts_service(JTS::JausRouter* jaus_router);
	virtual ~DigitalVideoPlugin_1_0();

	jVariableLengthString get_service_uri() { return "urn:jaus:jss:environmentSensing:DigitalVideo"; }
	jUnsignedByte get_version_number_major() { return 1; }
	jUnsignedByte get_version_number_minor() { return 0; }

protected:
	urn_jaus_jss_environmentSensing_DigitalVideo_1_0::DigitalVideoService *p_my_service;
	urn_jaus_jss_environmentSensing_VisualSensor_1_0::VisualSensorService *p_base_service;
	urn_jaus_jss_core_AccessControl_1_0::AccessControlService *p_accesscontrol_service;
	urn_jaus_jss_core_Events_1_0::EventsService *p_events_service;
	urn_jaus_jss_core_Transport_1_0::TransportService *p_transport_service;

};

};

#endif
