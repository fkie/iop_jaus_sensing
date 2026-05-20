This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## fkie_iop_digital_resource_discovery: DigitalResourceDiscovery

Service to manage the digital resource endpoint of a robot. This service should be run inside of a platform node and only once in a subsystem.

#### Parameter:

_delay_first_response (double_, Default: 5.0)

> Waits given time in seconds for start of other components. While this time all requests for DigitalResourceEndpoint will be ignored.


#### Publisher:

> None

#### Subscriber:

> None


## fkie_iop_digital_resource_discovery: DigitalResourceDiscoveryClient

This client offers methods to register digital endpoints by DigitalResourceDiscovery service. This is used e.g. by DigitalVideo service.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None
