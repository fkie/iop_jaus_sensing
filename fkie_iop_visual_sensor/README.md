This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_visual_sensor:_ VisualSensor

VisualSensor provides control features and names for sensor ID's of StillImage and DigitalVideo services. For each sensor you can set follow _capabilities_ parameter:

```
    capabilities:
        - 1.name.Front
        - 1.switchable.false
        - 1.zoomable.false
        - 1.pose.1 2 3 0 0 0 1
        - 1.manipulator.3.1.60
        - 1.manipulator.5
        - 1.fov_horizontal.0
        - 1.fov_vertical.0

```
Delete parameter for features which are not provided to disable.


#### Parameter:

_capabilities.ID.name (str_ (Default: ""))

> Name of the sensor.

_capabilities.ID.switchable (bool_ (Default: false))

> Enable publisher _ID/cmd_pwr_state_ and subscriber _ID/pwr_state_ to switch the sensor on/off.

_capabilities.ID.zoomable (bool_ (Default: false))

> Enable publisher _ID/cmd_zoom_level_ and subscriber _ID/zoom_level_ to change the zoom level of the sensor.

_capabilities.ID.pose (list_ (Default: invalid pose))

> A list with seven values for "x y z qx qy qz qw"

_capabilities.ID.manipulator (JAUS ID_ (Default: invalid ID))

> JAUS id for manipulator service.

_capabilities.ID.manipulator_joint (int_ (Default: invalid id))

> Joint number for manipulator joint.

_capabilities.ID.fov_horizontal (float_ (Default: invalid float))

> Horizontal field of view in radians. Creates also a subscriber _ID/fov_horizontal_ to receive changed fov values.

_capabilities.ID.fov_vertical (float_ (Default: invalid float))

> Vertical field of view in radians. Creates also a subscriber _ID/fov_vertical_ to receive changed fov values.

_sensor_names (list_, (Default: []))

#### Publisher:

_sensor\_`ID`/cmd_pwr_state (std_msgs::msg::Bool)_

> Sets the new power state for the sensor. Only available if switchable is `true`.

_sensor\_`ID`/cmd_zoom_level (std_msgs::msg::Bool)_

> Sets the new zoom level for the sensor. Only available if zoomable is `true`.


#### Subscriber:

_sensor\_`ID`/pwr_state (std_msgs::msg::Bool)_

> Reads the current power state of the sensor. Only available if switchable is `true`.

_sensor\_`ID`/zoom_level (std_msgs::msg::Bool)_

> Reads the current zoom level for the sensor. Only available if zoomable is `true`.

_sensor\_`ID`/fov_horizontal (std_msgs::msg::Float32)_

> Reads the current horizontal field of view for the sensor. Only available if fov_horizontal is set.

_sensor\_`ID`/fov_vertical (std_msgs::msg::Float32)_

> Reads the current vertical field of view for the sensor. Only available if fov_vertical is set.

