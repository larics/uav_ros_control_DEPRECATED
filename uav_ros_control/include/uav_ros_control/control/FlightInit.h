/*
 * FlightInit.h
 *
 *  Created on: Feb 5, 2020
 *      Author: AnaBatinovic
 */

#ifndef FLIGHT_INIT_H
#define FLIGHT_INIT_H
#include <iostream>
// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

#include <algorithm>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// Service for path/trajectory flags
#include <dynamic_reconfigure/server.h>
#include <uav_ros_control/FlightInitParametersConfig.h>
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <uav_ros_control_msgs/TakeOff.h>
#include <tf2/LinearMath/Quaternion.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <larics_motion_planning/MultiDofTrajectory.h>
#include <larics_motion_planning/MultiDofTrajectoryRequest.h>
#include <larics_motion_planning/MultiDofTrajectoryResponse.h>

#include <sensor_fusion_comm/InitHeight.h>
#include <sensor_fusion_comm/InitScale.h>

namespace flight_init 
{
/**
 * Name of dynamic reconfigure node.
 */
typedef uav_ros_control::FlightInitParametersConfig fi_param_t;
#define FLIGHT_INIT_DYN_RECONF  	"flight_init_config"
#define PARAM_TIME_FOR_INIT			"flight_init_node/time_for_init"
#define PARAM_RATE					"flight_init_node/rate"
#define PARAM_TAKEOFF_HEIGHT		"flight_init_node/takeoff_height"
#define PARAM_RADIUS_INIT			"flight_init_node/radius_init"
#define PARAM_EXECUTION_NUM         "flight_init_node/execute_trajectory_num"
#define PARAM_MAP_FRAME          	"flight_init_node/map_frame"  

#define PI 3.1415926535
#define DEGTORAD PI / 180.0

class FlightInit 
{
public: 
	/**
	 * Default constructor.
	*/
	FlightInit (ros::NodeHandle& nh)
	{
		initializeParameters(nh);

		// Initialize subscribers 
		m_subOdometry = nh.subscribe(
			"mavros/global_position/local", 1, 
			&flight_init::FlightInit::odometryCb, this);
		m_subState = nh.subscribe(
				"mavros/state", 1, 
				&flight_init::FlightInit::stateCb, this);
		m_subCartographerPose = nh.subscribe("uav/cartographer/pose", 1,
			&flight_init::FlightInit::cartographerPoseCb, this);
		m_subMsfOdometry = nh.subscribe("msf_core/odometry", 1,
			&flight_init::FlightInit::msfOdometryCb, this);

		// Initialize publishers 
		m_pubGoalsMarker = nh.advertise<visualization_msgs::MarkerArray>(
			"flight_init/goals_marker", 20);
		m_pubTrajectory = nh.advertise<trajectory_msgs::JointTrajectory> (
			"joint_trajectory", 1);
		m_pubReadyForExploration = nh.advertise<std_msgs::Bool> (
			"ready_for_exploration", 1);
	
		// Advertise service
    	m_serviceTakeOff = nh.advertiseService(
			"arm_and_takeoff", &flight_init::FlightInit::armAndTakeOffCb, this);
		
		// Clients
		m_armingClient = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    	m_setModeClient = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
		m_takeoffClient = nh.serviceClient<uav_ros_control_msgs::TakeOff>
			("takeoff");
		m_initializeMsfHeightClient = nh.serviceClient<sensor_fusion_comm::InitHeight>
			("msf_pose_sensor/pose_sensor/initialize_msf_height");
		m_initializeMsfScaleClient = nh.serviceClient<sensor_fusion_comm::InitScale>
			("msf_pose_sensor/pose_sensor/initialize_msf_scale");			
		m_planTrajectoryClient = nh.serviceClient<larics_motion_planning::MultiDofTrajectory> (
			"multi_dof_trajectory");
		
		// Setup dynamic reconfigure server
		fi_param_t  fiConfig;
		setReconfigureParameters(fiConfig);
		m_fiConfigServer.updateConfig(fiConfig);
		m_fiParamCallback = boost::bind(
			&flight_init::FlightInit::fiParamCb, this, _1, _2);
		m_fiConfigServer.setCallback(m_fiParamCallback);
	}

void initializeParameters(ros::NodeHandle& nh)
{
	ROS_INFO("FlightInit::initializeParameters()");
	bool initialized = 
			nh.getParam(PARAM_TIME_FOR_INIT, m_timeForInit)
	&& nh.getParam(PARAM_TAKEOFF_HEIGHT, m_takeoffHeight)
	&& nh.getParam(PARAM_RADIUS_INIT, m_radiusInit)
	&& nh.getParam(PARAM_EXECUTION_NUM, m_executeTrajectoryNum)
	&& nh.getParam(PARAM_MAP_FRAME, m_mapFrame)
			&& nh.getParam(PARAM_RATE, m_rate);

  ROS_INFO("Node rate: %.2f", m_rate);
  ROS_INFO("Time for initialization: %.2f", m_timeForInit);
	ROS_INFO("Radius around UAV for initialization: %.2f", m_radiusInit);
	ROS_INFO("Execution num: %d", m_executeTrajectoryNum);
	ROS_INFO("Takeoff height: %.2f", m_takeoffHeight);
  if (!initialized)
	{
		ROS_FATAL("FlightInit::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("FlightInit parameters not properly initialized.");
	}
}

void fiParamCb(fi_param_t& configMsg,uint32_t level)
{
	ROS_WARN("FlightInit::fiParamCb()");
  m_timeForInit = configMsg.time_for_init;
	m_takeoffHeight = configMsg.takeoff_height;
	m_radiusInit = configMsg.radius_init;
	m_executeTrajectoryNum = configMsg.execute_trajectory_num;

}

void setReconfigureParameters(fi_param_t& configMsg)
{
	ROS_WARN("Hello from setReconfigureParameters");
	configMsg.time_for_init = m_timeForInit;
	configMsg.takeoff_height = m_takeoffHeight;
	configMsg.radius_init =  m_radiusInit;
	configMsg.execute_trajectory_num = m_executeTrajectoryNum;
}

void stateCb(const mavros_msgs::State::ConstPtr& msg)
{
	m_currentState = *msg;
}

void cartographerPoseCb(const geometry_msgs::PoseStamped& msg)
{	
	std_msgs::Bool m_ready;
	// When we take off
	if (m_takeoffFlag && !m_mapInitializedFlag)
	{
			// First time previous = new
		if (first_time)
		{
			first_time = false;
			m_previousCartographerPose = msg;
			m_currentCartographerPose = msg;
			// Set timer for map initialization
			m_timer = ros::Time::now();
		}
		m_previousCartographerPose = m_currentCartographerPose;
		m_currentCartographerPose = msg;
		double distance = sqrt(
				pow(m_previousCartographerPose.pose.position.x
					- m_currentCartographerPose.pose.position.x, 2)
				+ pow(m_previousCartographerPose.pose.position.y
					- m_currentCartographerPose.pose.position.y, 2)
				+ pow(m_previousCartographerPose.pose.position.z
					- m_currentCartographerPose.pose.position.z, 2));
		
		if (distance > 1.0)
		{
			// Timer starts when init flight starts
			// Sudden shift, reset timer
			m_timer = ros::Time::now(); 
			m_previousCartographerPose = m_currentCartographerPose;
			ROS_WARN("Sudden shift!");
		}
		else 
		{
			if (ros::Time::now() - m_timer > ros::Duration(m_timeForInit))
			{
				// Enough time pass --> map is initialized
				ROS_INFO("Map initialized.");
				m_ready.data = true;
				m_mapInitializedFlag = true;
			}
			
		}
	m_pubReadyForExploration.publish(m_ready);	
	}
}

void msfOdometryCb(const nav_msgs::OdometryConstPtr& msg)
{
	ROS_INFO("msfOdometryCb");
	m_msfOdometryFlag = true;
}

bool armAndTakeOffCb(
	std_srvs::SetBool::Request& request, 
	std_srvs::SetBool::Response& response)
{
	const auto set_response = [&response] (bool success) { response.success = success; };

	if (!modeGuided())
	{
		ROS_FATAL("TakeoffCb - request denied, not in GUIDED_NOGPS");
		set_response(false);
		return true;
	}
	
	if (!armUAV())
	{
		 ROS_FATAL("TakeoffCb - request denied, ARMING failed.");
		set_response(false);
		return true;
	}

	ros::Duration(ARM_DURATION).sleep();

	if (!takeOffUAV())
	{
		ROS_FATAL("TakeoffCb - request denied, TAKEOFF unsuccessful");
		set_response(false);
		return true;
	}

	ros::Duration(TAKEOFF_DURATION).sleep();
	
	// Assume takeoff is successful at this point
	ROS_INFO("TakeoffCb - request approved, TAKEOFF successful");
	m_takeoffFlag = true;
	set_response(true);
	return true;
	
}

void odometryCb(const nav_msgs::OdometryConstPtr& msg)
{
	if (m_firstOdomFlag)
	{
		m_firstOdomFlag = false;
		m_homeOdom = *msg; 
	}	
	m_currentOdom = *msg;
}

bool modeGuided()
{
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "GUIDED";
	
	if (m_currentState.mode != "GUIDED")
	{
		if (m_setModeClient.call(offb_set_mode))
		{
			ros::Duration(2.0).sleep();
			if (offb_set_mode.response.mode_sent)
			{
			std::cout << "STATE: " << m_currentState.mode << std::endl;
			ROS_INFO ("modeGuided - GUIDED_NOGPS enabled");
			return true;
			}
		ROS_FATAL("modeGuided - Setting mode GUIDED_NOGPS failed.");
		return false;
		}
		ROS_FATAL("modeGuided - Setting mode GUIDED_NOGPS failed.");
		return false;
	}
	ROS_WARN("modeGuided - GUIDED_NOGPS mode already set.");
	return true;
} 

bool armUAV()
{	
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	if (!m_currentState.armed)
	{
		// Call arming
		if (m_armingClient.call(arm_cmd))
			ros::Duration(0.5).sleep();
		{
			if (arm_cmd.response.success)
			{
				ROS_INFO("armUAV - Vehicle armed");
				return true;
			}
			ROS_FATAL("armUAV - Calling arming failed.");
			return false;
		}
		ROS_FATAL("armUAV - Calling arming failed.");
		return false;
	}
	ROS_WARN("armUAV - Already armed.");
	return true;
}

bool takeOffUAV()
{
	// Call takeoff 
	uav_ros_control_msgs::TakeOff take_off;
	take_off.request.rel_alt = m_takeoffHeight;

	if (m_takeoffClient.call(take_off))
	{
		ros::Duration(0.2).sleep();
		if (take_off.response.success)
		{
			ROS_INFO("takeOffUAV - Takeoff successfully called.");
			return true;
		}
		ROS_FATAL("takeOffUAV - Takeoff response failed.");
		return false;
	}
	ROS_FATAL("takeOffUAV - Takeoff call failed.");
	return false;
}

void startInitFlight()
{
	m_vectorWaypoints = {};
	generateWaypoints(m_vectorWaypoints);

	// Check if waypoints are generated
	if (m_vectorWaypoints.size() == 0)
	{
		ROS_FATAL("startFlightCb: waypoints are not generated.");
		m_startFlightFlag = false;
	} 
	if (!publishTrajectory(m_vectorWaypoints))
	{
		ROS_FATAL("startInitFlight - publishing trajectory failed.");
		m_startFlightFlag = false;

	}
	else
	{
		ROS_INFO("startInitFlight - publishing trajectory successful.");
		m_startFlightFlag = true;
	}
}

void generateWaypoints(
	std::vector<geometry_msgs::Point> &m_vectorWaypoints)
{
	geometry_msgs::Point m_point;
	double m_circumference = 2 * m_radiusInit * PI;
	double m_k = PI;
	// Round down value
	int m_points_num = int(ceil(m_circumference / m_k));
	std::cout << m_points_num << std::endl;
	double m_segment = 360 / m_points_num;
	// Current UAV position
	geometry_msgs::Point m_current_position = m_currentOdom.pose.pose.position;
	// CIRCLE around UAV
	for (int i = 0; i < m_points_num; i++)
	{
		m_point.x = m_current_position.x + 
			(m_radiusInit * cos(i * m_segment * DEGTORAD));
		m_point.y = m_current_position.y + 
			(m_radiusInit * sin(i * m_segment * DEGTORAD));
		m_point.z = m_takeoffHeight;
		m_vectorWaypoints.push_back(m_point);
	}	
}

double quaternion2Yaw(geometry_msgs::Quaternion quaternion)
{
	double q0 = quaternion.w;
	double q1 = quaternion.x;
	double q2 = quaternion.y;
	double q3 = quaternion.z;
	return atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}

bool publishTrajectory (
	std::vector<geometry_msgs::Point> &m_vectorOfPoints)	
{
	larics_motion_planning::MultiDofTrajectory m_srv;
	// Save home position to calculate orientation
	geometry_msgs::Point m_home_position;
	m_home_position.x = m_homeOdom.pose.pose.position.x;
	m_home_position.y = m_homeOdom.pose.pose.position.y;
	m_home_position.z = m_homeOdom.pose.pose.position.z;

	trajectory_msgs::JointTrajectoryPoint m_trajectory_point;
	// Create start point from the current position information
	std::vector<double> m_points_arr {
		m_currentOdom.pose.pose.position.x,
		m_currentOdom.pose.pose.position.y, 
		m_currentOdom.pose.pose.position.z,
		quaternion2Yaw(m_currentOdom.pose.pose.orientation)};
	m_trajectory_point.positions.clear();
	m_trajectory_point.positions = m_points_arr;
	m_srv.request.waypoints.points.push_back(m_trajectory_point);

	for (int k = 0; k < m_executeTrajectoryNum; k++)
	{
		// Create another points from generated vector of points
		for (int i = 0; i < m_vectorOfPoints.size() - 1; i++)
		{	
			m_points_arr = {
				m_vectorOfPoints[i].x,
				m_vectorOfPoints[i].y, 
				m_vectorOfPoints[i].z,
				atan2(( m_vectorOfPoints[i+1].y - m_vectorOfPoints[i].y),
				( m_vectorOfPoints[i+1].x - m_vectorOfPoints[i].x))};
			m_trajectory_point.positions.clear();
			m_trajectory_point.positions = m_points_arr;
			m_srv.request.waypoints.points.push_back(m_trajectory_point);
		}
		
	// Append last point with orientation from the first point
	m_points_arr = {
		m_vectorOfPoints[m_vectorOfPoints.size()-1].x,
		m_vectorOfPoints[m_vectorOfPoints.size()-1].y, 
		m_vectorOfPoints[m_vectorOfPoints.size()-1].z,
		atan2(( m_vectorOfPoints[1].y - m_vectorOfPoints[0].y),
		( m_vectorOfPoints[1].x - m_vectorOfPoints[0].x))};
	m_trajectory_point.positions.clear();
	m_trajectory_point.positions = m_points_arr;
	m_srv.request.waypoints.points.push_back(m_trajectory_point);
	
	m_srv.request.publish_path = false;
	m_srv.request.publish_trajectory = false;
	m_srv.request.plan_path = false;
	m_srv.request.plan_trajectory = true;
	}
	// Call the service
	bool m_service_succes = m_planTrajectoryClient.call(m_srv);

	if (m_service_succes && m_srv.response.success)
	{
		ROS_INFO("publishTrajectory - MultiDOFTrajectory service successfully called.");
		trajectory_msgs::JointTrajectory m_generated_trajectory = 
			m_srv.response.trajectory; 
		// Publish trajectory
		m_pubTrajectory.publish(m_generated_trajectory);
		return true;
	
	}
	else
	{
		ROS_FATAL("publishTrajectory - MultiDOFTrajectory service call failed.");
		return false; 
	}
}

void msfInitializeHeight()
{
	sensor_fusion_comm::InitHeight m_init_height;
	m_init_height.request.height = m_currentCartographerPose.pose.position.z;
	// Call msf init height

	if (m_initializeMsfHeightClient.call(m_init_height))
	{
		ros::Duration(0.2).sleep();
		//std::cout << "Msf response: "<< m_init_height.response.result << std::endl;
		
		ROS_INFO("msfInitHeight: successfully called.");
		m_msfInitializedHeightFlag = true;
	}
	else
	{
		ROS_FATAL("msfInitHeight: call failed.");
		m_msfInitializedHeightFlag = true;
	}
}


void msfInitializeScale()
{
	sensor_fusion_comm::InitScale m_init_scale;
	m_init_scale.request.scale = 1.0;
	// Call msf init height

	if (m_initializeMsfScaleClient.call(m_init_scale))
	{
		ros::Duration(0.2).sleep();
		
		ROS_INFO("msfInitScale: successfully called.");
		m_msfInitializedScaleFlag = true;
	}
	else
	{
		ROS_FATAL("msfInitScale: call failed.");
		m_msfInitializedScaleFlag = true;
	}
}

void startMission()
{
	// In every iteration check
	if (m_takeoffFlag)
	{
		if (!m_startFlightFlag)
		{
			startInitFlight();
			ROS_WARN("START MISSION: publishing trajectory called.");
		}
		if (m_mapInitializedFlag && !m_msfInitializedHeightFlag)
		{
			// Map is initialized and octomap server is called
			msfInitializeHeight();
			ros::Duration(2.0).sleep();
			if (!m_msfInitializedScaleFlag)
			{
				msfInitializeScale();
			}
		}
		if (m_msfOdometryFlag)
		{
			ROS_INFO("START MISSION: msf odometry publishing.");
		}
	}
}

void run()
{
  ros::Rate loopRate(m_rate);
	// Call service to set path and trajectory flags
	while (ros::ok())
	{
		ros::spinOnce();
		startMission();
    loopRate.sleep();
	}
}

private: 
double m_timeForInit, m_takeoffHeight, m_rate, m_radiusInit;
std_msgs::Int32 m_executingTrajectory;
int m_executeTrajectoryNum;
mavros_msgs::State m_currentState;
sensor_msgs::NavSatFix m_currentGlobalPosition;
geometry_msgs::Point m_currGoal;
geometry_msgs::PoseStamped m_previousCartographerPose, m_currentCartographerPose;
nav_msgs::Odometry m_currentOdom, m_homeOdom;
ros::Subscriber m_subState, m_subOdometry, m_subCartographerPose, m_subMsfOdometry;
ros::Publisher m_pubGoalsMarker, m_pubTrajectory, m_pubReadyForExploration;
ros::Time m_timer;
bool m_takeoffFlag = false;
bool m_startFlightFlag = false;
bool m_firstOdomFlag = true;
bool first_time = true;
bool m_mapInitializedFlag = false;
bool m_msfInitializedHeightFlag = false;
bool m_msfInitializedScaleFlag = false;
bool m_msfOdometryFlag = false;
std::string m_mapFrame;
std::vector<geometry_msgs::Point> m_vectorWaypoints = {};
ros::ServiceServer m_serviceTakeOff, m_serviceStartFlight;
ros::ServiceClient m_armingClient, m_setModeClient, m_takeoffClient,
m_setTrajectoryFlagsClient, m_startFlightClient, m_planTrajectoryClient, 
m_initializeMsfHeightClient, m_initializeMsfScaleClient;
/* Define Dynamic Reconfigure parameters */
boost::recursive_mutex m_fiConfigMutex;
dynamic_reconfigure::Server<fi_param_t>
	m_fiConfigServer {m_fiConfigMutex, ros::NodeHandle(FLIGHT_INIT_DYN_RECONF)};
dynamic_reconfigure::Server<fi_param_t>::CallbackType m_fiParamCallback;

static constexpr double ARM_DURATION = 3.0;
static constexpr double TAKEOFF_DURATION = 15.0;
};

}

#endif /* FLIGHT_INIT_H */