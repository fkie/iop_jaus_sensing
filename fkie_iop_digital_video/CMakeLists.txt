cmake_minimum_required(VERSION 3.5)
project(fkie_iop_digital_video)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
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
    urn.jaus.jss.core-v1.1/AccessControl.xml
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
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
  ${fkie_iop_accesscontrol_INCLUDE_DIRS}
  ${fkie_iop_digital_resource_discovery_INCLUDE_DIRS}
  ${fkie_iop_discovery_INCLUDE_DIRS}
  ${fkie_iop_visual_sensor_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            ${cppfiles}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${std_msgs_TARGETS}
  fkie_iop_accesscontrol::fkie_iop_accesscontrol
  fkie_iop_digital_resource_discovery::fkie_iop_digital_resource_discovery
  fkie_iop_discovery::fkie_iop_discovery
  fkie_iop_visual_sensor::fkie_iop_visual_sensor
)


#############
## Install ##
#############

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS}
  DESTINATION include/${PROJECT_NAME}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
   DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(
  include
  ${IOP_COMPONENT_INCLUDE_DIRS}
  ${fkie_iop_accesscontrol_INCLUDE_DIRS}
  ${fkie_iop_digital_resource_discovery_INCLUDE_DIRS}
  ${fkie_iop_discovery_INCLUDE_DIRS}
  ${fkie_iop_visual_sensor_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
