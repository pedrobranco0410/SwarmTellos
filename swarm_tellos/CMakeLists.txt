cmake_minimum_required(VERSION 3.5)
project(swarm_tellos)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()



file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf")
foreach (INDEX RANGE 0 8)
  if (${INDEX} EQUAL 0)
    set(SUFFIX "")
    set(TOPIC_NS "solo")
  else ()
    set(SUFFIX "_${INDEX}")
    set(TOPIC_NS "drone${INDEX}")
  endif ()
  set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/urdf/tello${SUFFIX}.urdf")
  message(STATUS "creating rules for ${URDF_FILE}")
  add_custom_command(
    OUTPUT ${URDF_FILE}
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/urdf/replace.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/tello.xml" "suffix=${SUFFIX}" "topic_ns=${TOPIC_NS}" ">" "${URDF_FILE}"
    DEPENDS urdf/tello.xml
    COMMENT "Generate ${URDF_FILE}"
    VERBATIM
  )
  add_custom_target(generate_urdf${SUFFIX} ALL DEPENDS ${URDF_FILE})
endforeach ()



# Find packages
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_shared_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${swarm_msgs_INCLUDE_DIRS}
)


#=============
# Install
#=============

# Install targets
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf"
  DESTINATION share/${PROJECT_NAME}
)

# Install all Python programs
install(
  PROGRAMS  src/drone_controller.py
  src/drone_controller_gazebo.py
  src/global_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install various directories
install(
  DIRECTORY
  launch
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
