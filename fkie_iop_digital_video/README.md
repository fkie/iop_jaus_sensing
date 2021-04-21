This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_digital_video:_ DigitalVideo

Offers an interface to register video sources by DigitalResourceDiscovery service. Currently no configuration for registered videos source supported.

#### Parameter:

_endpoints (list_, (Default: [])

> Specifies a list with video sources. An entry have follow structure: 
```ID.TYPE.URL```, e.g.: ```- 1.rtsp.http://address:port```.
ID is a ressource id of range {1..65534}. 0 and 65534 are reserved. If you include a StillImage service into your component do not use the same ID. These ID's are used in VisualSensor to merge for a name of the sensor. TYPE represents a server type, a value of {rtsp_topic, rtsp, http, https, ftp, sftp, ftp_ssh, scp, mpeg2ts}. URL is a path of address of the video.


#### Publisher:

_dv_resource_id (std_msgs::UInt16)_, latched

> Publishes the last played stream sent by JAUS message ControlDigitalVideoSensorStream. On Stop command a value of 65535 is published.

#### Subscriber:

> None
