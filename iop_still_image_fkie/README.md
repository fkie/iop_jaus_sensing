This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_still_image_fkie:_ StillImage

Translate CompressedImage to IOP network. 

#### Parameter:

_image_sensors (list_, (Default: [])

> List of pairs of type {ID: ROS topic name}, e.g.: ```"4": map_image/compressed``` The topics must have a type of sensor_msgs::CompressedImage. If you include a DigitalVideo service into your component do not use the same ID. These ID's are used in VisualSensor to merge for a name of the sensor.

#### Publisher:

> None

#### Subscriber:

> _{topics specified in image_sensors}_ (sensor_msgs::CompressedImage)
