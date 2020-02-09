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

		// Define subscribers 
		m_subOdometry = nh.subscribe(
			"mavros/global_position/local", 1, 
			&flight_init::FlightInit::odometryCb, this);

		m_subState = nh.subscribe<mavros_msgs::State>(
				"mavros/state", 1, 
				&flight_init::FlightInit::stateCb, this);

		m_subGlobalPosition = nh.subscribe<sensor_msgs::NavSatFix>(
			"mavros/global_position/global", 10, 
			&flight_init::FlightInit::globalPositionCb, this);
		// m_subCartographerPose = nh.subscribe ("uav/cartographer/pose", 1,
		// 	&flight_init::FlightInit::cartographerPosecb, this);

		m_pubGoalsMarker = nh.advertise<visualization_msgs::MarkerArray>(
			"flight_init/goals_marker", 20);
		m_pubTrajectory = nh.advertise<trajectory_msgs::JointTrajectory> (
			"joint_trajectory", 1);
	
		// Services
    	m_serviceTakeOff = nh.advertiseService(
			"arm_and_takeoff", &flight_init::FlightInit::takeOffCb, this);
		m_serviceStartFlight = nh.advertiseService(
			"start_flight", &flight_init::FlightInit::startFlightCb, this);
	
		m_armingClient = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    	m_setModeClient = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
		m_takeoffClient = nh.serviceClient<uav_ros_control_msgs::TakeOff>
			("takeoff");
		m_startFlightClient = nh.serviceClient<std_srvs::SetBool> (
			"start_flight");
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
		&& nh.getParam(PARAM_MAP_FRAME, m_mapFrame)
        && nh.getParam(PARAM_RATE, m_rate);

    ROS_INFO("Node rate: %.2f", m_rate);
    ROS_INFO("Time for initialization: %.2f", m_timeForInit);
	 ROS_INFO("Radius around UAV for initialization: %.2f", m_radiusInit);
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

}

void setReconfigureParameters(fi_param_t& configMsg)
{
	ROS_WARN("Hello from setReconfigureParameters");
	configMsg.time_for_init = m_timeForInit;
	configMsg.takeoff_height = m_takeoffHeight;
	configMsg.radius_init =  m_radiusInit;
}

void stateCb(const mavros_msgs::State::ConstPtr& msg)
{
	m_currentState = *msg;
}

void globalPositionCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	m_currentGlobalPosition = *msg;
}

// void cartographerPosecb(const geometry_msgs::PoseStamped& msg)
// {
// 	m_cartographerPose = *msg;
// }

bool takeOffCb(
	std_srvs::SetBool::Request& request, 
	std_srvs::SetBool::Response& response)
{

	ROS_INFO("TakeOff service called.");
	m_serviceTakeoffCalledFlag = true;
	
	response.success = true;
	response.message = "TakeOff service called.";
	return true;
}

bool startFlightCb(
	std_srvs::SetBool::Request& request, 
	std_srvs::SetBool::Response& response)
{
	m_vectorWaypoints = {};
	generateWaypoints(m_vectorWaypoints);

	ROS_INFO("Start flight service called.");
	// Set timer for map initialization
	m_timer = ros::Time::now();
	m_serviceStartFlightCalledFlag = true;
	
	response.success = true;
	response.message = "Start flight service called.";
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

void pointReachedCb(const std_msgs::Bool msg)
{
	m_isPointReached = msg.data;
	ROS_INFO("Current point is reached.");
}

bool modeGuided()
{
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "GUIDED_NOGPS";
	
	if (m_currentState.mode != "GUIDED_NOGPS")
	{
		if (m_setModeClient.call(offb_set_mode))
		{
			ros::Duration(2.0).sleep();
			if (offb_set_mode.response.mode_sent)
			{
			std::cout << "STATE: " << m_currentState.mode << std::endl;
			ROS_INFO ("GUIDED_NOGPS enabled");
			return true;
			}
		ROS_FATAL("Setting mode GUIDED_NOGPS failed.");
		return false;
		}
		ROS_FATAL("Setting mode GUIDED_NOGPS failed.");
		return false;
	}
	ROS_WARN("GUIDED_NOGPS mode already set.");
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
				ROS_INFO("Vehicle armed");
				return true;
			}
			ROS_FATAL("Calling arming failed.");
			return false;
		}
		ROS_FATAL("Calling arming failed.");
		return false;
	}
	ROS_WARN("Already armed.");
	return true;
}

bool takeOffUAV()
{
	// Call takeoff 
	uav_ros_control_msgs::TakeOff take_off;
	take_off.request.rel_alt = m_takeoffHeight;
	ros::Duration(2.0).sleep();
	if (m_takeoffClient.call(take_off))
	{
		ros::Duration(0.2).sleep();
		if (take_off.response.success)
		{
			ROS_INFO("Takeoff successfully called.");
			return true;
		}
		ROS_INFO("Takeoff response failed.");
		return false;
	}
	ROS_INFO("Takeoff call failed.");
	return false;
}

bool startInitFlight()
{
	// Call service for init flight
	std_srvs::SetBool start_init_flight;
	start_init_flight.request.data = true;

	if (m_startFlightClient.call(start_init_flight))
	{
		ros::Duration(0.2).sleep();
		if (start_init_flight.response.success)
		{
			ROS_INFO("Service start_flight called.");
			return true;	
		}
		ROS_FATAL("Calling start_flight failed.");
		return false;
	}	
	ROS_FATAL("Calling start_flight failed.");
	return false;
}

void generateWaypoints(
	std::vector<geometry_msgs::Point> &m_vectorWaypoints)
{
	// ROS_INFO("Generating waypoints");
	geometry_msgs::Point m_point;
	double m_circumference = 2 * m_radiusInit * PI;
	// Define segment on circle (10 points, 5 m radius)
	double m_k = PI;
	double m_points_num = m_circumference / m_k;
	double m_segment = 360 / m_points_num;
	// Current UAV position
	geometry_msgs::Point m_current_position = m_currentOdom.pose.pose.position;
	// CIRCLE around UAV
	for (int i = 0; i < (int)round(m_points_num); i++)
	{
		m_point.x = m_current_position.x + 
			(m_radiusInit * cos(i * m_segment * DEGTORAD));
		m_point.y = m_current_position.y + 
			(m_radiusInit * sin(i * m_segment * DEGTORAD));
		m_point.z = m_takeoffHeight;
		m_vectorWaypoints.push_back(m_point);
	}
	visualizeGeneratedWaypoints(m_vectorWaypoints, m_points_num);	
}


void visualizeGeneratedWaypoints(
	std::vector<geometry_msgs::Point> m_vectorWaypoints, double num)
{
	visualization_msgs::MarkerArray Markerarr;
	visualization_msgs::Marker Marker;
	Markerarr.markers.resize(num);

  for (int i = 0; i < m_vectorWaypoints.size(); i++)
  {
    Marker.header.frame_id = m_mapFrame;
    Marker.header.stamp = ros::Time::now();
	Marker.id = i;
    Marker.ns = "init_goals";
    Marker.type = visualization_msgs::Marker::SPHERE;
	Marker.pose.position.x = m_vectorWaypoints[i].x;
    Marker.pose.position.y = m_vectorWaypoints[i].y;
	Marker.pose.position.z = m_vectorWaypoints[i].z;
	Marker.pose.orientation.x = 0;
	Marker.pose.orientation.y = 0;
	Marker.pose.orientation.z = 0;
	Marker.pose.orientation.w = 1;
	
	Marker.scale.x = 0.25;
    Marker.scale.y = 0.25;
    Marker.scale.z = 0.1;
    Marker.color.a = 1.0;
    Marker.color.r = 0.0;
    Marker.color.g = 1.0;
    Marker.color.b = 0.0;
	Marker.lifetime = ros::Duration(360);

	Markerarr.markers.push_back(Marker);
 
	ROS_INFO("Publishing markers!");
				
	m_pubGoalsMarker.publish(Markerarr);
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

void publishTrajectory (
	std::vector<geometry_msgs::Point> &m_vectorOfPoints)	
{
	ROS_INFO("enter publish trajectory");
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
	
	// Call the service
	bool m_service_succes = m_planTrajectoryClient.call(m_srv);
	
	if (m_service_succes && m_srv.response.success)
	{
		ROS_INFO("MultiDOFTrajectory service successfully called.");
		trajectory_msgs::JointTrajectory m_generated_trajectory = 
			m_srv.response.trajectory; 
		// Publish trajectory
		ROS_WARN("Publish trajectory.");
		m_pubTrajectory.publish(m_generated_trajectory);
		m_serviceStartFlightCalledFlag = false;
		
	}
	else
	{
		ROS_FATAL("MultiDOFTrajectory service call failed.");
		//TODO: edit 
		//m_serviceStartFlightCalledFlag = true;
	}
}

void checkMapInitialization(bool &initialized)
{
	// m_cartographerPose --> current pose in map
}

void run()
{
    ros::Rate loopRate(m_rate);
	// Call service to set path and trajectory flags
	while (ros::ok())
	{
		ros::spinOnce();
		if (m_serviceTakeoffCalledFlag)
		{	
			// Enable guided no gps 
			m_serviceTakeoffCalledFlag = false;
			bool guided = modeGuided();
			if (guided)
			{
				
				bool armed = armUAV();

				if (armed)
				{
					// Try calling service from uav_ros_control
					//std::cout << "response: " << int(m_takeoff.response.result) << std::endl;
					// if (m_takeoffClient.call(m_takeoff) &&
					// 	int(m_takeoff.response.success))
					// {
					bool takeoff_success = takeOffUAV(); 
					if (takeoff_success)
					{
						m_takeoffFlag = true;
					}
					else
					{
						ROS_INFO ("Takeoff failed: Calling arm_and_takeoff service again");
						m_serviceTakeoffCalledFlag = true;
					}
					
				}
				else
				{
					ROS_INFO ("Arm failed: Calling arm_and_takeoff service again");
					m_serviceTakeoffCalledFlag = true;
				}
				
			}
			else
			{
				ROS_INFO ("Mode GUIDED_NOGPS failed: Calling arm_and_takeoff service again");
				m_serviceTakeoffCalledFlag = true;
			}
			
		
		}
		// Call start flight 	
		if (m_takeoffFlag && m_serviceStartFlightCalledFlag)
		{
			// Init flight start
			publishTrajectory(m_vectorWaypoints);
		}

        loopRate.sleep();
	}

		
}


private: 

double m_timeForInit, m_takeoffHeight, m_rate, m_radiusInit;
int position_in_vector = 0;
mavros_msgs::State m_currentState;
sensor_msgs::NavSatFix m_currentGlobalPosition;
geometry_msgs::Point m_currGoal;
geometry_msgs::PoseStamped m_cartographerPose;
nav_msgs::Odometry m_currentOdom, m_homeOdom;
ros::Subscriber m_subState, m_subGlobalPosition, m_subOdometry, m_subPointReached, m_subCartographerPose;
ros::Publisher m_pubGoalsMarker, m_pubTrajectory;
ros::Time m_timer;
bool m_serviceTakeoffCalledFlag = false;
bool m_takeoffFlag = false;
bool m_serviceStartFlightCalledFlag = false;
bool m_isPointReached = true;
bool m_publishedTrajectory = false;
bool m_firstOdomFlag = true;
std::string m_mapFrame;
std::vector<geometry_msgs::Point> m_vectorWaypoints = {};
std::vector<geometry_msgs::Point> m_vectorWaypointsInit = {};
ros::ServiceServer m_serviceTakeOff, m_serviceStartFlight;
ros::ServiceClient m_armingClient, m_setModeClient, m_takeoffClient,
m_setTrajectoryFlagsClient, m_startFlightClient,m_planTrajectoryClient;
/* Define Dynamic Reconfigure parameters */
boost::recursive_mutex m_fiConfigMutex;
dynamic_reconfigure::Server<fi_param_t>
	m_fiConfigServer {m_fiConfigMutex, ros::NodeHandle(FLIGHT_INIT_DYN_RECONF)};
dynamic_reconfigure::Server<fi_param_t>::CallbackType m_fiParamCallback;

};

}

#endif /* FLIGHT_INIT_H */