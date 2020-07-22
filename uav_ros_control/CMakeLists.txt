cmake_minimum_required(VERSION 2.8.3)
project(uav_ros_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	uav_ros_control_msgs
	topp_ros
	rospy
    std_msgs
    dynamic_reconfigure
    sensor_msgs
	mav_msgs
	mavros
	geometry_msgs
	trajectory_msgs
	message_generation
	tf)


list(INSERT CMAKE_MODULE_PATH 0 "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

## Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

find_package(cmake_modules REQUIRED)

generate_dynamic_reconfigure_options(
	config/control/PositionControlParameters.cfg
	config/control/DistanceControlParameters.cfg
	config/control/FlightInitParameters.cfg
	config/reference/GenerateSearchParameters.cfg
	config/reference/PositionKalmanFilterParameters.cfg
	config/reference/VisualServoParameters.cfg
	config/reference/VisualServoStateMachineParameters.cfg
	config/reference/VisualServoPursuitParameters.cfg
)

add_service_files(
  FILES
  GenerateSearch.srv
  GenerateInterception.srv
  GiveMeBalloonPosition.srv
  GetLocalConstraints.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES pid nfilt kalman
	CATKIN_DEPENDS uav_ros_control_msgs topp_ros)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${GeographicLib_INCLUDE_DIRS})

# PID controller library
add_library(pid src/control/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

# Nonlinear filters library
add_library(nfilt src/filters/NonlinearFilters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})

# Kalman filter library
add_library(kalman src/filters/KalmanFilter.cpp)
target_link_libraries(kalman ${catkin_LIBRARIES})

# References library
file(GLOB_RECURSE reference_SRC "src/reference/*.cpp")
add_library(reference_lib ${reference_SRC})
target_link_libraries(reference_lib ${catkin_LIBRARIES} nfilt pid yaml-cpp)

# Controller library
file(GLOB_RECURSE controllers_SRC "src/control/*.cpp")
add_library(control_lib ${controllers_SRC})
target_link_libraries(control_lib ${catkin_LIBRARIES} nfilt)

# Carrot node
add_executable(carrot_reference_node nodes/reference/CarrotReferenceNode.cpp)
target_link_libraries(carrot_reference_node reference_lib)

# Visual servo node
add_executable(visual_servo_node nodes/reference/VisualServoNode.cpp)
target_link_libraries(visual_servo_node reference_lib)

# Track and Follow state machine
add_executable(track_and_follow_sm nodes/reference/TrackAndFollowStateNode.cpp)
target_link_libraries(track_and_follow_sm ${catkin_LIBRARIES} nfilt reference_lib ${GeographicLib_LIBRARIES})

# Wait and Observe state machine
add_executable(wait_and_observe_sm nodes/reference/WaitAndObserveStateNode.cpp)
target_link_libraries(wait_and_observe_sm ${catkin_LIBRARIES} nfilt reference_lib ${GeographicLib_LIBRARIES})

# PID cascade node
add_executable(pid_cascade_node nodes/control/PositionControlNode.cpp)
target_link_libraries(pid_cascade_node control_lib)

# Distance control node
add_executable(distance_control_node nodes/control/DistanceControlNode.cpp)
target_link_libraries(distance_control_node control_lib nfilt)

# Flight init node
add_executable(flight_init_node nodes/control/FlightInitNode.cpp)
target_link_libraries(flight_init_node control_lib nfilt)

# Antonella's geo-fencing node
add_executable(geo_fence_local_check_node nodes/reference/GeoFenceLocalCheck.cpp)
target_link_libraries(geo_fence_local_check_node reference_lib ${catkin_LIBRARIES} nfilt)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
