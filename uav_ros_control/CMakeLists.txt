cmake_minimum_required(VERSION 2.8.3)
project(uav_ros_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	uav_ros_control_msgs
	rospy
    std_msgs
    dynamic_reconfigure
    sensor_msgs
	mav_msgs
	tf)

find_package(cmake_modules REQUIRED)

generate_dynamic_reconfigure_options(
	config/control/PositionControlParameters.cfg
	config/control/DistanceControlParameters.cfg
	config/reference/VisualServoParameters.cfg
	config/reference/VisualServoStateMachineParameters.cfg
	config/reference/VisualServoPursuitParameters.cfg
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES pid nfilt kalman
	CATKIN_DEPENDS uav_ros_control_msgs)

include_directories(
	include
	${catkin_INCLUDE_DIRS})

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
target_link_libraries(reference_lib ${catkin_LIBRARIES} nfilt pid)

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

# visual servo state machine
add_executable(visual_servo_sm nodes/reference/VisualServoStateNode.cpp)
target_link_libraries(visual_servo_sm ${catkin_LIBRARIES} nfilt)

# visual servo PURSUIT state machine
add_executable(visual_servo_sm_pursuit nodes/reference/VisualServoStatePursuitNode.cpp)
target_link_libraries(visual_servo_sm_pursuit ${catkin_LIBRARIES} nfilt reference_lib)

# PID cascade node
add_executable(pid_cascade_node nodes/control/PositionControlNode.cpp)
target_link_libraries(pid_cascade_node control_lib)

# Distance control node
add_executable(distance_control_node nodes/control/DistanceControlNode.cpp)
target_link_libraries(distance_control_node control_lib nfilt)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
