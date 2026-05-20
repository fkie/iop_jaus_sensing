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


#ifndef DIGITAL_RESOURCE_ENDPOINT_H
#define DIGITAL_RESOURCE_ENDPOINT_H

#include <string>
#include "Transport/JausAddress.h"

namespace digital_resource_endpoint
{

const int SERVER_TYPE_INVALID  = -1;
const int SERVER_TYPE_RTSP     = 0;
const int SERVER_TYPE_MPEG2TS  = 1;
const int SERVER_TYPE_FTP      = 2;
const int SERVER_TYPE_SFTP     = 3;
const int SERVER_TYPE_FTP_SSH  = 4;
const int SERVER_TYPE_HTTP     = 5;
const int SERVER_TYPE_HTTPS    = 6;
const int SERVER_TYPE_SCP      = 7;

class DigitalResourceEndpoint
{
 public:
  DigitalResourceEndpoint(int server_type=SERVER_TYPE_INVALID, std::string server_url="", JausAddress iop_id=JausAddress(), unsigned short int resource_id=0, unsigned char request_id=0);
  virtual ~DigitalResourceEndpoint();

  // Server type, one of values of SERVER_TYPE_*
  int server_type;
  // URL of the digital resource server. This URL should not require a DNS to
  // resolve; hence, an IP address should be substituted for a host name.
  std::string server_url;
  // JAUS ID of the component that hosts any configuration and control services for this stream.
  JausAddress iop_id;
  // The ID used by the configuration and control service to identify this
  // source. This is the SensorID for visual sensors.
  unsigned short int resource_id;
  // Client provided ID to link the response to the request
  unsigned char request_id;

  bool isValid();
  bool operator==(const DigitalResourceEndpoint &value) const;
  bool operator!=(const DigitalResourceEndpoint &value) const;
};

}

#endif // DIGITAL_RESOURCE_ENDPOINT_H
