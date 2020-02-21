 /*
 * @author Robert Milijas
 * @version 0.1
 * @date December 2019
 *
 */

#ifndef BALLOON_STATE_MACHINE_H
#define BALLOON_STATE_MACHINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include "uav_ros_control/reference/Global2Local.h"

 namespace uav_reference
{


enum States {
    TAKE_OFF,
    SEARCH,
    POPPING
};

class BalloonStateMachine
{

public:

BalloonStateMachine(ros::NodeHandle& nh)
{
    _pubVssmState = nh.advertise<std_msgs::Int32>("visual_servo_sm/state", 1);
    _pubSearchTrajectoryFlag = nh.advertise<std_msgs::Bool>("/topp/trajectory_flag", 1);

    // Define Subscribers
    _subOdom = nh.subscribe("odometry", 1, &uav_reference::PursuitStateMachine::odomCb, this);
    /* UAV vision based errors */

    // Setup Pursuit service callback
    _servicePursuit = nh.advertiseService("pursuit", &uav_reference::PursuitStateMachine::pursuitServiceCb, this);

    // Initialize visual servo client caller
    _vsClienCaller = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("visual_servo");

    // Initialize search trajectory caller
    _searchTrajectoryClientCaller = nh.serviceClient<uav_ros_control::GenerateSearch>("generate_search");

    std::pair<double, double> tmp_local_waypoint;
    auto tmp_local_waypoint = Global2Local::toLocal()

}

~BalloonStateMachine()
{}


void requestSearchTrajectory(){

    // Todo request trajectory along the predefined GPS coordinates


    // Todo koji je alt? Je li uopće bitan?
    _local_start = Global2Local::toLocal(_lat_start, _long_start, 0);
    _local_mid_1 = Global2Local::toLocal(_lat_mid_1, _long_mid_1, 0);
    _local_mid_2 = Global2Local::toLocal(_lat_mid_2, _long_mid_2, 0);
    _local_end = Global2Local::toLocal(_lat_end, _long_end, 0);




    ROS_INFO("PursuitSM::update status - requesting search trajectory.");
    uav_ros_control::GenerateSearch srv;
    srv.request.desired_height = _search_height;
    srv.request.x_size = _arena_x_size;
    srv.request.y_size = _arena_y_size;
    srv.request.x_offset = _arena_x_offset;
    srv.request.y_offset = _arena_y_offset;
    srv.request.yaw_offset = _arena_yaw_offset;
    srv.request.x_takeOff = _x_takeOff;
    srv.request.y_takeOff = _y_takeOff;
    srv.request.uav_yaw = _search_uav_yaw;

    if(!_searchTrajectoryClientCaller.call(srv)){
        ROS_FATAL("PursuitSM::updateStatus - search trajectory not generated.");
        _currentState = PursuitState::OFF;
        return;
    }

    if (srv.response.success){
        ROS_INFO("PursuitSM::updateStatus - search trajectory successfully generated.");
    }
}

void updateState()
{
    // TODO: DO state machine logic here
    checkDetection();

    // 1
    // If visual servo is inactive, deactivate state machine
    // Visual servo can be inactive in state OFF and SEARCH.
    if (_currentState != PursuitState::OFF && !_pursuitActivated)
    {
        ROS_WARN("PursuitSM::updateStatus - Visual servo is inactive.");
        _currentState = PursuitState::OFF;
        turnOffVisualServo();
        _searchTrajectoryFlag.data = false;
        // requestSearchTrajectory();
        ROS_WARN("PursuitSM::updateStatus - OFF State activated.");
        return;
    }
    // Request search trajectory
    if (_currentState == PursuitState::OFF && (!_start_following_uav | !_isDetectionActive) && _pursuitActivated){
        ROS_WARN("PursuitSM::updateStatus - SEARCH state activated.");
        turnOffVisualServo();
        _currentState = PursuitState::SEARCH;
        _searchTrajectoryFlag.data = true;
        requestSearchTrajectory();
    }
    // Activate Pursuit algorithm when detection is confident.
    if ((_currentState == PursuitState::OFF | _currentState == PursuitState::SEARCH )&& _start_following_uav && _isDetectionActive && _pursuitActivated)
    {
        ROS_INFO("PursuitSM::updateStatus - Starting visual servo for UAV following.");
        _currentState = PursuitState::UAV_FOLLOWING;
        _searchTrajectoryFlag.data = false;
        turnOnVisualServo();

        // Comment this out for testing pursposes
        //_currHeightReference = 0;
        //_currDistanceReference = _uav_distance_offset;

        _currDistanceReference = _relativeUAVDistance;

        _currHeightReference = _relativeUAVHeight;
        _currYawReference = _relativeUAVYaw;

        if (isRelativeDistanceNan()){
            _currDistanceReference = _maxDistanceReference;
            ROS_WARN("PursuitSM::updateStatus - UAV distance is Nan.");
        }
        else if (!isRelativeDistancePositive()) {
            // pass
            ROS_FATAL("PursuitSM::updateStatus - UAV distance is negative.");
            _currDistanceReference = _uav_distance_offset;
        }

        return;
    }
    // Transition to ball following when distance is below certain threshold.
    // If detection is not confident anymore or visual servo is deactivated or detection node is inactive, turn off UAV following.
    if (_currentState == PursuitState::UAV_FOLLOWING && (!_start_following_uav || !_pursuitActivated || !_isDetectionActive))
    {
        ROS_WARN("PursuitSM::updateStatus - exiting UAV_FOLLOWING mode.");
        ROS_WARN_COND(!_start_following_uav, "PursuitSM::condition - UAV following not confident anymore.");
        ROS_WARN_COND(!_pursuitActivated, "PursuitSM::condition - service Pursuit is not active anymore.");
        ROS_WARN_COND(!_isDetectionActive, "PursuitSM::condition - detection is inactive.");

        turnOffVisualServo();
        _currentState = PursuitState::OFF;
        //requestSearchTrajectory();
        ROS_WARN("PursuitSM::updateStatus - OFF State activated.");
        return;
    }

    if (_currentState == PursuitState::UAV_FOLLOWING && _pursuitActivated)
    {
        // Comment this out for testing pursposes
        //_currHeightReference = 0;
        //_currDistanceReference = _uav_distance_offset;

        _currDistanceReference = _relativeUAVDistance;

        _currHeightReference = _relativeUAVHeight;
        _currYawReference = _relativeUAVYaw;

        if (isRelativeDistanceNan()){
            _currDistanceReference = _maxDistanceReference;
            ROS_WARN("PursuitSM::updateStatus - UAV distance is Nan.");
        }
        else if (!isRelativeDistancePositive()) {
            // pass
            ROS_FATAL("PursuitSM::updateStatus - UAV distance is negative.");
            _currDistanceReference = _uav_distance_offset;
        }

        return;
    }
}


void run()
{
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;
	while (ros::ok())
	{
		ros::spinOnce();
        updateState();
        _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
        loopRate.sleep();
    }
}

private:

    double _rate = 50;

    /* Service Pursuit */
	ros::ServiceServer _servicePursuit;
    bool _pursuitActivated = false;
    PursuitState _currentState = PursuitState::OFF;

    /* Client for calling visual servo and search trajectory */
    ros::ServiceClient _vsClienCaller, _searchTrajectoryClientCaller;

    /* Offset subscriber and publisher */
    ros::Publisher _pubVssmState, _pubOffsetY, _pubOffsetZ;

    /* Trajectory */
    ros::Publisher _pubSearchTrajectoryFlag;
    std_msgs::Bool _searchTrajectoryFlag;

    /* Error publishers */
    ros::Publisher _pubXError, _pubYError, _pubZError, _pubYawError;

    /* Error subscribers */
    ros::Subscriber _subUAVDist, _subBALLDist;
    ros::Subscriber _subUAVHeightError, _subBALLHeightError;
    ros::Subscriber _subUAVYawError, _subBALLYawError;
    double _relativeUAVDistance = INVALID_DISTANCE, _relativeBALLDistance = INVALID_DISTANCE;
    double _relativeUAVHeight,_relativeUAVYaw = 0;

    /* Confidence subscribers */
    ros::Subscriber _subUAVPursuitConfident, _subBALLPursuitConfident;
    ros::Subscriber _subUAVDistanceConfident;

    /* Pose publisher */
    ros::Publisher _pubVisualServoFeed;
    uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;
    nav_msgs::Odometry _currOdom;

    /* Parameters */
    double _currDistanceReference, _currHeightReference, _currYawReference;
    bool _start_following_uav = false, _isDetectionActive = false, _kf_distance_active = false;
    float _uav_distance_offset, _ball_distance_offset;
    float _uav_z_offset;
    ros::Time _time_last_detection_msg;
    float _maxDistanceReference = 15.0;
    float _arena_x_size, _arena_y_size, _arena_x_offset, _arena_y_offset, _arena_yaw_offset;
    double _x_takeOff, _y_takeOff, _search_height;
    double _search_uav_yaw;

    // Todo: set as param

    const double _lat_start = 24.41772, _long_start = 54.43562;
    const double _lat_mid_1 = 24.41767, _long_mid_1 = 54.43582;
    const double _lat_mid_2 = 24.4176, _long_mid_2 = 54.4361;
    const double _lat_end = 24.41756, _long_end = 54.43626;
    const double _global_alt;

    std::vector<std::pair<double>> _global_waypoints{
            {_lat_start, _long_start},
            {_lat_mid_1, _long_mid_1},
            {_lat_mid_2, _long_mid_2},
            {_lat_end, _long_end}
    };

    std::queue<std::pair<double,double>> _local_waypoints;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _pursuitConfigMutex;
    dynamic_reconfigure::Server<pursuit_param_t>
        _pursuitConfigServer {_pursuitConfigMutex, ros::NodeHandle(PURSUIT_DYN_RECONF)};
    dynamic_reconfigure::Server<pursuit_param_t>::CallbackType _pursuitParamCallback;
};
}

#endif /* VISUAL_SERVO_PURSUIT_STATE_MACHINE_H*/
