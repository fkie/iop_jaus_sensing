See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:

[iop_digital_video_fkie: DigitalVideo](#iop_digital_video_fkie-digitalvideo)  
[iop_range_sensor_fkie: RangeSensor](#iop_range_sensor_fkie-rangesensor)  
[iop_still_image_fkie: StillImage](#iop_still_image_fkie-stillimage)  
[iop_visual_sensor_fkie: VisualSensor](#iop_visual_sensor_fkie-visualsensor)

## _iop_digital_video_fkie:_ DigitalVideo

Offers an interface to register video sources by DigitalResourceDiscovery service. Currently no configuration for registered videos source supported.

#### Parameter:

_video_endpoints (list_, (Default: [])

> Specifies a list with video sources. An entry have follow structure: 
```ID/TYPE: URL```, e.g.: ```"0/rtsp" : "http://address:port"```.
ID is a ressource id of range {1..65534}. 0 and 65534 are reserved. If you include a StillImage service into your component do not use the same ID. These ID's are used in VisualSensor to merge for a name of the sensor. TYPE represents a server type, a value of {rtsp_topic, rtsp, http, https, ftp, sftp, ftp_ssh, scp, mpeg2ts}. URL is a path of address of the video.


#### Publisher:

_dv_resource_id (std_msgs::UInt16)_, latched

> Publishes the last played stream sent by JAUS message ControlDigitalVideoSensorStream. On Stop command a value of 65535 is published.

#### Subscriber:

> None

## _iop_range_sensor_fkie:_ RangeSensor

Offers an interface to register video sources by DigitalResourceDiscovery service. Currently no configuration for registered videos source supported.

#### Parameter:

_range_sensors (list_, (Default: [])

> List of string with topic names. The topics must have a type of sensor_msgs::LaserScan.

_tf_frame_robot (str_, (Default: "base_link")

> ROS Tf to set the ReportSensorGeometricProperties.

#### Publisher:

> None

#### Subscriber:

> _{topics specified in sensors}_ (sensor_msgs::LaserScan)

## _iop_still_image_fkie:_ StillImage

Translate CompressedImage to IOP network. 

#### Parameter:

_image_sensors (list_, (Default: [])

> List of pairs of type {ID: ROS topic name}, e.g.: ```"4": map_image/compressed``` The topics must have a type of sensor_msgs::CompressedImage. If you include a DigitalVideo service into your component do not use the same ID. These ID's are used in VisualSensor to merge for a name of the sensor.

#### Publisher:

> None

#### Subscriber:

> _{topics specified in image_sensors}_ (sensor_msgs::CompressedImage)

## _iop_visual_sensor_fkie:_ VisualSensor

The current functionality is limited to hold names for sensor ID's of StillImage and DigitalVideo services. 

#### Parameter:

_sensor_names (list_, (Default: [])

> List of pairs of type {ID: Name}, e.g.: ```"3": Map```. ID is a ressource id of range {1..65534}. 0 and 65534 are reserved. Name is a string with maximal length of 255 chars.

#### Publisher:

> None

#### Subscriber:

> None

