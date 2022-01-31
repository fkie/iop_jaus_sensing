cmake_minimum_required(VERSION 3.5)
project(fkie_iop_digital_video)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# set default c++ version
if($ENV{ROS_DISTRO} MATCHES "foxy")
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
elseif($ENV{ROS_DISTRO} MATCHES "galactic")
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()


find_package(ament_cmake REQUIRED)
find_package(fkie_iop_accesscontrol REQUIRED)
find_package(fkie_iop_digital_resource_discovery REQUIRED)
find_package(fkie_iop_discovery REQUIRED)
find_package(fkie_iop_visual_sensor REQUIRED)
find_package(std_msgs REQUIRED)

###############
## setup IOP ##
###############
iop_init()
iop_export_service(
  urn_jaus_jss_environmentSensing_DigitalVideo
)
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.0/AccessControl.xml
    urn.jaus.jss.core-v1.0/Events.xml
    urn.jaus.jss.core-v1.0/EventsClient.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.environmentSensing/DigitalVideo.xml
    urn.jaus.jss.environmentSensing/VisualSensor.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_environmentSensing_DigitalVideo/DigitalVideo_ReceiveFSM.h
    src/urn_jaus_jss_environmentSensing_DigitalVideo/DigitalVideo_ReceiveFSM.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_AccessControl
    urn_jaus_jss_core_Transport
    urn_jaus_jss_environmentSensing_VisualSensor
  GENERATED_SOURCES cppfiles
)


###########
## Build ##
###########
set(dependencies
  fkie_iop_accesscontrol
  fkie_iop_digital_resource_discovery
  fkie_iop_discovery
  fkie_iop_visual_sensor
  std_msgs
)

include_directories(
  ${IOP_COMPONENT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            ${cppfiles}
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Install ##
#############

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS}
  DESTINATION include/${PROJECT_NAME}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
   DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
