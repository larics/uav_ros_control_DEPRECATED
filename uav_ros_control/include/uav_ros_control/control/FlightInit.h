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
            // m_subCartographerPose = nh.subscribe("uav/cartographer/pose", 1,
            // 	&flight_init::FlightInit::cartographerPoseCb, this);
            // m_subMsfOdometry = nh.subscribe("msf_core/odometry", 1,
            // 	&flight_init::FlightInit::msfOdometryCb, this);

            // Initialize publishers
            m_pubReadyForExploration = nh.advertise<std_msgs::Bool> (
                    "ready_for_exploration", 1, true);

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
            // m_initializeMsfHeightClient = nh.serviceClient<sensor_fusion_comm::InitHeight>
            // 	("msf_pose_sensor/pose_sensor/initialize_msf_height");
            // m_initializeMsfScaleClient = nh.serviceClient<sensor_fusion_comm::InitScale>
            // 	("msf_pose_sensor/pose_sensor/initialize_msf_scale");
            // m_planTrajectoryClient = nh.serviceClient<larics_motion_planning::MultiDofTrajectory> (
            // 	"multi_dof_trajectory");

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
            bool initialized = nh.getParam(PARAM_TAKEOFF_HEIGHT, m_takeoffHeight);

            ROS_INFO("Takeoff height: %.2f", m_takeoffHeight);
            if (!initialized)
            {
                ROS_FATAL("FlightInit::initializeParameters() - failed to initialize parameters");
                throw std::runtime_error("FlightInit parameters not properly initialized.");
            }
        }

        bool healthyNumberOfPublishers()
        {
            ROS_FATAL_COND(!m_subOdometry.getNumPublishers() > 0, "IF - 'mavros odometry' topic publisher missing");
            ROS_FATAL_COND(!m_subState.getNumPublishers() > 0, "IF - 'mavros state' topic publisher missing");
            // ROS_FATAL_COND(!m_subCartographerPose.getNumPublishers() > 0, "IF - 'cartographer pose' topic publisher missing");

            return m_subOdometry.getNumPublishers() > 0
                   && 	m_subState.getNumPublishers() > 0;
            // && 	m_subCartographerPose.getNumPublishers() > 0;
        }


        bool all_services_available()
        {
            ROS_FATAL_COND(!m_armingClient.exists(), "FI - arming service does not exist.");
            ROS_FATAL_COND(!m_setModeClient.exists(), "FI - set mode service does not exist.");
            ROS_FATAL_COND(!m_takeoffClient.exists(), "FI - takeoff service does not exist.");
            // ROS_FATAL_COND(!m_initializeMsfHeightClient.exists(), "FI - msf height service does not exist.");
            // ROS_FATAL_COND(!m_initializeMsfScaleClient.exists(), "FI - msf scale service does not exist.");
            // ROS_FATAL_COND(!m_planTrajectoryClient.exists(), "FI - multi_dof_trajectory does not exist.");

            return m_armingClient.exists()
                   && m_setModeClient.exists()
                   && m_takeoffClient.exists();
            // && m_initializeMsfHeightClient.exists()
            // && m_initializeMsfScaleClient.exists()
            // && m_planTrajectoryClient.exists();
        }

        void fiParamCb(fi_param_t& configMsg,uint32_t level)
        {
            ROS_WARN("FlightInit::fiParamCb()");
            m_takeoffHeight = configMsg.takeoff_height;

        }

        void setReconfigureParameters(fi_param_t& configMsg)
        {
            ROS_WARN("Hello from setReconfigureParameters");
            configMsg.takeoff_height = m_takeoffHeight;
        }

        void stateCb(const mavros_msgs::State::ConstPtr& msg)
        {
            m_currentState = *msg;
        }

        void rcInCallback(const mavros_msgs::RCIn& msg)
        {
            m_rcInput = msg.channels[4];
        }

        bool armAndTakeOffCb(
                std_srvs::SetBool::Request& request,
                std_srvs::SetBool::Response& response)
        {
            std_msgs::Bool t_ready_msg;
            t_ready_msg.data = false;

            const auto set_response = [&response] (bool success) { response.success = success; };
            if (!healthyNumberOfPublishers() || !all_services_available())
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

            ros::Duration(TAKEOFF_DURATION).sleep();
            // Ensure last odom msg
            t_ready_msg.data = true;
            m_pubReadyForExploration.publish(t_ready_msg);
            ros::spinOnce();
            // Open the GRASPER
            // std_msgs::String msg;
            // msg.data = "unwinding";
            // m_pubDynamixelState.publish(msg);
            // ros::spinOnce();
            // ros::Duration(GRASPER_DURATION).sleep();
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
            ros::Rate loopRate(20);
            // Call service to set path and trajectory flags
            double t_0 = ros::Time::now().toSec();
            while (ros::ok())
            {
                ros::spinOnce();
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
        static constexpr double TAKEOFF_DURATION = 10.0;
        static constexpr double GRASPER_DURATION = 30.0;
    };

}

#endif /* FLIGHT_INIT_H */