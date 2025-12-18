cmake_minimum_required(VERSION 3.10)
project(tracker)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    message(WARNING "tracker: OpenVR library only supports x86_64 architecture")
    install(FILES package.xml DESTINATION share/${PROJECT_NAME})
    ament_package()
    return()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(OPENVR_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr)
set(OPENVR_INCLUDE_DIRS ${OPENVR_ROOT_DIR}/headers)

find_library(OPENVR_LIBRARIES NAMES openvr_api
  PATHS
    /opt/steamvr/bin/vrwebhelper/linux64
    /opt/steamvr/bin/linux64
  NO_DEFAULT_PATH)

if(NOT OPENVR_LIBRARIES)
  set(OPENVR_LIBRARIES ${OPENVR_ROOT_DIR}/lib/linux64/libopenvr_api.so)
endif()

get_filename_component(OPENVR_LIB_DIR "${OPENVR_LIBRARIES}" DIRECTORY)

add_library(tracker_pipeline SHARED
  src/pipeline/validator.cpp
  src/pipeline/detector.cpp
  src/pipeline/calibrator.cpp
  src/pipeline/filter.cpp
)
target_include_directories(tracker_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tracker_pipeline
  PUBLIC Eigen3::Eigen
  PRIVATE yaml-cpp
)

add_library(tracker_core SHARED
  src/openvr_context.cpp
  src/tracker_reader.cpp
  src/steamvr_manager.cpp
)
target_include_directories(tracker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OPENVR_INCLUDE_DIRS}
)
target_link_libraries(tracker_core
  PRIVATE ${OPENVR_LIBRARIES}
  PUBLIC Eigen3::Eigen tracker_pipeline
)
set_target_properties(tracker_core PROPERTIES
  INSTALL_RPATH "${OPENVR_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
  VERSION 1.0.0
  SOVERSION 1
  LINK_FLAGS "-Wl,--exclude-libs,ALL"
)

add_executable(trackers_node src/trackers_node.cpp)
add_executable(steamvr_node src/steamvr_node.cpp)

add_executable(list_trackers tools/list_tracker_sn.cpp)
add_executable(config_tracker_sn tools/config_tracker_sn.cpp)

set(CPP_DEPENDENCIES
  rclcpp
  std_srvs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

ament_target_dependencies(trackers_node ${CPP_DEPENDENCIES})
target_link_libraries(trackers_node tracker_core tracker_pipeline)

ament_target_dependencies(steamvr_node rclcpp)
target_link_libraries(steamvr_node tracker_core)

target_include_directories(list_trackers PRIVATE ${OPENVR_INCLUDE_DIRS})
target_link_libraries(list_trackers PRIVATE ${OPENVR_LIBRARIES})

target_include_directories(config_tracker_sn PRIVATE ${OPENVR_INCLUDE_DIRS})
target_link_libraries(config_tracker_sn PRIVATE ${OPENVR_LIBRARIES})

set_target_properties(trackers_node steamvr_node list_trackers config_tracker_sn PROPERTIES
  INSTALL_RPATH "${OPENVR_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(trackers_node PRIVATE -O3)
  target_compile_options(steamvr_node PRIVATE -O3)
  target_compile_options(list_trackers PRIVATE -O3)
  target_compile_options(config_tracker_sn PRIVATE -O3)
endif()

install(TARGETS tracker_pipeline tracker_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS trackers_node steamvr_node list_trackers config_tracker_sn
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()
