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
#include <mavros_msgs/RCIn.h>
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
		m_subRcIn = nh.subscribe("mavros/rc/in", 1,
			&flight_init::FlightInit::rcInCallback, this);

		// Initialize publishers 
		m_pubTrajectory = nh.advertise<trajectory_msgs::JointTrajectory> (
			"joint_trajectory", 1);
		m_pubReadyForExploration = nh.advertise<std_msgs::Bool> (
			"ready_for_exploration", 1, true);
		m_pubDynamixelState = nh.advertise<std_msgs::String>("/dynamixel_workbench/set_state", 1);
	
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
    bool initialized = nh.getParam(PARAM_TIME_FOR_INIT, m_timeForInit)
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

bool subscribedTopicsActive() 
{
    double currentTime = ros::Time::now().toSec();
    double dt_odom = currentTime - m_timeLastOdometry;
    double dt_state = currentTime - m_timeLastState;
    double dt_cartographer = currentTime - m_timeLastCartographer;
    
    static constexpr double MAX_DT = 0.2;

    return true;
}

bool healthyNumberOfPublishers() 
{
    ROS_FATAL_COND(!m_subOdometry.getNumPublishers() > 0, "IF - 'mavros odometry' topic publisher missing");
    ROS_FATAL_COND(!m_subState.getNumPublishers() > 0, "IF - 'mavros state' topic publisher missing");

    return m_subOdometry.getNumPublishers() > 0 
      && 	m_subState.getNumPublishers() > 0;
}

bool stateMachineDisableConditions()
{
    return !subscribedTopicsActive();
}

bool all_services_available()
{
	ROS_FATAL_COND(!m_armingClient.exists(), "FI - arming service does not exist.");
	ROS_FATAL_COND(!m_setModeClient.exists(), "FI - set mode service does not exist.");
	ROS_FATAL_COND(!m_takeoffClient.exists(), "FI - takeoff service does not exist.");

  return m_armingClient.exists() 
    && m_setModeClient.exists()
    && m_takeoffClient.exists();
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
	m_timeLastState = ros::Time::now().toSec();
}

void rcInCallback(const mavros_msgs::RCIn& msg)
{
	m_rcInput = msg.channels[4]; 
}

void dynamixelStateCb(){

}

bool armAndTakeOffCb(
	std_srvs::SetBool::Request& request, 
	std_srvs::SetBool::Response& response)
{
  	std_msgs::Bool t_ready_msg;
	t_ready_msg.data = false;

	const auto set_response = [&response] (bool success) { response.success = success; };
	if (stateMachineDisableConditions() || !healthyNumberOfPublishers() || !all_services_available())
  {

    if (!healthyNumberOfPublishers())
		{
      ROS_FATAL("IF::armAndTakeoffCb - check connected publishers.");
			set_response(false);
			m_pubReadyForExploration.publish(t_ready_msg);
			return true;
		}
		if (!all_services_available())
		{
      ROS_FATAL("IF::armAndTakeoffCb - check services.");
			set_response(false);
			m_pubReadyForExploration.publish(t_ready_msg);
			return true;			
		}
	}

	if (!modeGuided())
	{
		ROS_FATAL("TakeoffCb - request denied, not in GUIDED_NOGPS");
		set_response(false);
		m_pubReadyForExploration.publish(t_ready_msg);
		return true;
	}
	
	if (!armUAV())
	{
		ROS_FATAL("TakeoffCb - request denied, ARMING failed.");
		set_response(false);
		m_pubReadyForExploration.publish(t_ready_msg);
		return true;
	}

	ros::Duration(ARM_DURATION).sleep();

	if (!takeOffUAV())
	{
		ROS_FATAL("TakeoffCb - request denied, TAKEOFF unsuccessful");
		set_response(false);
		m_pubReadyForExploration.publish(t_ready_msg);
		return true;
	}
	// Sleep before grasper
	ros::Duration(5.0).sleep();
	// Open the GRASPER
	std_msgs::String msg;
	msg.data = "unwinding";
	m_pubDynamixelState.publish(msg);
	// Sleep to stabilize before State Machine
	//ros::Duration(20.0).sleep();
	// Tell SM that TakeOff is successful
	t_ready_msg.data = true;
	m_pubReadyForExploration.publish(t_ready_msg);
	ros::spinOnce();
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
	m_timeLastOdometry = ros::Time::now().toSec();
}

bool modeGuided()
{
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "GUIDED_NOGPS";
	std::cout << "Rc in: " << m_rcInput << std::endl;
	if (m_rcInput > 1750)
	{
		if (m_currentState.mode != "GUIDED_NOGPS")
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
	ROS_FATAL("mode Guided, RC/IN - Can not enter GUIDED_NOGPS mode. Check futaba!");
	return false;
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

void generateWaypoints(
	std::vector<geometry_msgs::Point> &m_vectorWaypoints)
{
	geometry_msgs::Point m_point;
	m_point.z = m_currentOdom.pose.pose.position.z;
	std::cout<<"Mpoint z: "<<m_point.z<<std::endl;
	std::cout<<"odom z: "<<m_currentOdom.pose.pose.position.z<<std::endl;
	// Current UAV position
	geometry_msgs::Point m_current_position = m_currentOdom.pose.pose.position;
	double angle_inc = 180 / m_radiusInit;
	// CIRCLE around UAV
	for (double angle = 0; angle < m_executeTrajectoryNum * 360; angle += angle_inc)
	{
		m_point.x = m_current_position.x + 
			(m_radiusInit * cos(angle * DEGTORAD));
		m_point.y = m_current_position.y + 
			(m_radiusInit * sin(angle * DEGTORAD));
		m_vectorWaypoints.push_back(m_point);
	}
	m_point.x = m_current_position.x;
	m_point.y = m_current_position.y;
	m_vectorWaypoints.push_back(m_point);
	std::cout << "vector:" << m_vectorWaypoints.size() << std::endl;

	for (int i = 0; i < m_vectorWaypoints.size(); i++)
	{
		std::cout<<"Vector z: "<<m_vectorWaypoints[i].z<<std::endl;
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

void run()
{
  ros::Rate loopRate(m_rate);
	// Call service to set path and trajectory flags
	double t_0 = ros::Time::now().toSec();
	while (ros::ok())
	{
		ros::spinOnce();
		// startMission();
    loopRate.sleep();
	}
}

private: 
double m_timeForInit, m_takeoffHeight, m_rate, m_radiusInit;
double m_timeLastOdometry = 0,
  m_timeLastState = 0,
  m_timeLastCartographer = 0;
std_msgs::Int32 m_executingTrajectory;
int m_executeTrajectoryNum;
int m_rcInput;
mavros_msgs::State m_currentState;
sensor_msgs::NavSatFix m_currentGlobalPosition;
geometry_msgs::Point m_currGoal;
geometry_msgs::PoseStamped m_previousCartographerPose, m_currentCartographerPose;
nav_msgs::Odometry m_currentOdom, m_homeOdom;
ros::Subscriber m_subState, m_subOdometry, m_subCartographerPose, m_subMsfOdometry,
	m_subRcIn;
ros::Publisher m_pubDynamixelState;
ros::Publisher m_pubTrajectory, m_pubReadyForExploration;
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
static constexpr double TAKEOFF_DURATION = 30.0;
static constexpr double GRASPER_DURATION = 30.0;
};

}

#endif /* FLIGHT_INIT_H */