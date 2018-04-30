This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


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