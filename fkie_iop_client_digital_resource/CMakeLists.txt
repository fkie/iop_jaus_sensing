cmake_minimum_required(VERSION 3.5)
project(fkie_iop_client_digital_resource)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(fkie_iop_digital_resource_discovery REQUIRED)
find_package(fkie_iop_msgs REQUIRED)
find_package(fkie_iop_ocu_slavelib REQUIRED)

###############
## setup IOP ##
###############
iop_init()

###########
## Build ##
###########
set(dependencies
  fkie_iop_digital_resource_discovery
  fkie_iop_msgs
  fkie_iop_ocu_slavelib
)

include_directories(
  ${IOP_COMPONENT_INCLUDE_DIRS}
  ${fkie_iop_digital_resource_discovery_INCLUDE_DIRS}
  ${fkie_iop_ocu_slavelib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            src/DigitalResourceClient.cpp
            ${cppfiles}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${fkie_iop_msgs_TARGETS}
  fkie_iop_digital_resource_discovery::fkie_iop_digital_resource_discovery
  fkie_iop_ocu_slavelib::fkie_iop_ocu_slavelib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

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
  ${fkie_iop_digital_resource_discovery_INCLUDE_DIRS}
  ${fkie_iop_ocu_slavelib_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
