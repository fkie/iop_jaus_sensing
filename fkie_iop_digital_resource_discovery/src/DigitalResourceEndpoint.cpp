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

#include <fkie_iop_digital_resource_discovery/DigitalResourceEndpoint.h>


namespace digital_resource_endpoint
{

DigitalResourceEndpoint::DigitalResourceEndpoint(int server_type, std::string server_url, JausAddress iop_id, unsigned short int resource_id, unsigned char request_id)
{
  this->server_type = server_type;
  this->server_url = server_url;
  this->iop_id = iop_id;
  this->resource_id = resource_id;
  this->request_id = request_id;

}
DigitalResourceEndpoint::~DigitalResourceEndpoint()
{
}

bool DigitalResourceEndpoint::isValid()
{
  return (server_type != SERVER_TYPE_INVALID);
}

bool DigitalResourceEndpoint::operator==(const DigitalResourceEndpoint &value) const
{
  if (server_type != value.server_type) {
    return false;
  }

  if (server_url != value.server_url) {
    return false;
  }

  if (iop_id != value.iop_id) {
    return false;
  }

  if (resource_id != value.resource_id) {
    return false;
  }
  return true;
}

bool DigitalResourceEndpoint::operator!=(const DigitalResourceEndpoint &value) const
{
  return !((*this) == value);
}

}
