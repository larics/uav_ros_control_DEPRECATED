 /*
 * @author Antonella Barisic
 * @version 0.1
 * @date December 2019
 *
 */

#ifndef VISUAL_SERVO_PURSUIT_STATE_MACHINE_H
#define VISUAL_SERVO_PURSUIT_STATE_MACHINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_control/VisualServoPursuitParametersConfig.h>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <uav_ros_control/GenerateSearch.h>
#include <uav_ros_control/GenerateInterception.h>

namespace uav_reference 
{

typedef uav_ros_control::VisualServoPursuitParametersConfig pursuit_param_t;
#define PURSUIT_DYN_RECONF              "pursuit_state_machine"
#define PARAM_RATE                      "pursuit/state_machine/rate"
#define PARAM_UAV_DISTANCE_OFFSET       "pursuit/state_machine/uav_distance_offset"
#define PARAM_BALL_DISTANCE_OFFSET      "pursuit/state_machine/ball_distance_offset"
#define INVALID_DISTANCE -1
#define PARAM_Z_OFFSET                  "pursuit/state_machine/z_offset"
#define PARAM_YAW_ERROR_DEADZONE        "pursuit/state_machine/yaw_deadzone"
#define PARAM_ARENA_X_SIZE              "pursuit/state_machine/search/x_size"
#define PARAM_ARENA_Y_SIZE              "pursuit/state_machine/search/y_size"
#define PARAM_ARENA_X_OFFSET            "pursuit/state_machine/search/x_offset"
#define PARAM_ARENA_Y_OFFSET            "pursuit/state_machine/search/y_offset"
#define PARAM_ARENA_YAW_OFFSET          "pursuit/state_machine/search/yaw_offset"
#define PARAM_SEARCH_HEIGHT             "pursuit/state_machine/search/desired_height"
#define PARAM_X_TAKEOFF                 "pursuit/state_machine/search/x_takeOff"
#define PARAM_Y_TAKEOFF                 "pursuit/state_machine/search/y_takeOff"
#define PARAM_INTERCEPTION_Z_OFFSET     "pursuit/state_machine/interception/z_offset"

enum PursuitState {
    OFF,
    SEARCH,
    UAV_FOLLOWING,
    INTERCEPTION,
    BALL_GRASPING
};

class PursuitStateMachine
{

public:

PursuitStateMachine(ros::NodeHandle& nh)
{   
	initializeParameters(nh);
    // Define Publishers
    _pubVisualServoFeed = nh.advertise<uav_ros_control_msgs::VisualServoProcessValues>("visual_servo/process_value", 1);
    _pubOffsetY = nh.advertise<std_msgs::Float32>("visual_servo/offset_y", 1);
    _pubOffsetZ = nh.advertise<std_msgs::Float32>("visual_servo/offset_z", 1);
    _pubVssmState = nh.advertise<std_msgs::Int32>("visual_servo_sm/state", 1);
    _pubXError = nh.advertise<std_msgs::Float32>("sm_pursuit/x_err", 1);
    _pubYError = nh.advertise<std_msgs::Float32>("sm_pursuit/y_err", 1);
    _pubZError = nh.advertise<std_msgs::Float32>("sm_pursuit/z_err", 1);
    _pubYawError = nh.advertise<std_msgs::Float32>("sm_pursuit/yaw_err", 1);
    _pubSearchTrajectoryFlag = nh.advertise<std_msgs::Bool>("topp/trajectory_flag", 1);
    /* Detection activation/deactivation */
    _pubInferenceEnabled = nh.advertise<std_msgs::Bool>("/sm_pursuit/inference_enabled", 1);

    // Define Subscribers
    _subOdom = nh.subscribe("odometry", 1, &uav_reference::PursuitStateMachine::odomCb, this);
    /* UAV vision based errors */
    _subUAVDist = nh.subscribe("/uav_object_tracking/uav/distance_kf", 1, &uav_reference::PursuitStateMachine::uavDistCb, this);
    _subUAVHeightError = nh.subscribe("/uav_object_tracking/uav/height_kf", 1, &uav_reference::PursuitStateMachine::uavHeightCb, this);
    _subUAVYawError = nh.subscribe("/uav_object_tracking/uav/yaw_kf", 1, &uav_reference::PursuitStateMachine::uavYawCb, this);
    _subUAVPursuitConfident = nh.subscribe("/YOLODetection/uav_following_confident", 1, &uav_reference::PursuitStateMachine::uavConfidentCb, this);
    _subUAVDistanceConfident = nh.subscribe("/YOLODetection/kf_distance_active", 1, &uav_reference::PursuitStateMachine::uavDistanceConfidentCb, this);
    /* BALL vision based errors */
    _subBALLDist = nh.subscribe("/uav_object_tracking/ball/distance", 1, &uav_reference::PursuitStateMachine::ballDistCb, this);
    _subBALLHeightError = nh.subscribe("/uav_object_tracking/ball/height_error", 1, &uav_reference::PursuitStateMachine::ballHeightCb, this);
    _subBALLYawError = nh.subscribe("/uav_object_tracking/ball/yaw_error", 1, &uav_reference::PursuitStateMachine::ballYawCb, this);
    _subBALLPursuitConfident = nh.subscribe("/red_ball/confident", 1, &uav_reference::PursuitStateMachine::ballConfidentCb, this);
    /* Figure 8 estimator*/
    _subInterceptionPoint = nh.subscribe("target_uav/setpoint_estimated", 1, &uav_reference::PursuitStateMachine::figureEstimatorCb, this);
    _subCurrentEstimatedTargetPoint = nh.subscribe("/red/target_uav/position_estimated", 1, &uav_reference::PursuitStateMachine::currentEstimatedTargetPointCb, this);
    _subToppStatus = nh.subscribe("topp/status", 1, &uav_reference::PursuitStateMachine::toppStatusCb, this);


    // Setup dynamic reconfigure server
	pursuit_param_t  pursuitConfig;
	setPursuitParameters(pursuitConfig);
	_pursuitConfigServer.updateConfig(pursuitConfig);
	_pursuitParamCallback = boost::bind(
		&uav_reference::PursuitStateMachine::pursuitParamCb, this, _1, _2);
	_pursuitConfigServer.setCallback(_pursuitParamCallback);

    // Setup Pursuit service callback
    _servicePursuit = nh.advertiseService("pursuit", &uav_reference::PursuitStateMachine::pursuitServiceCb, this);

    // Initialize visual servo client caller
    _vsClienCaller = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("visual_servo");

    // Initialize search trajectory caller
    _searchTrajectoryClientCaller = nh.serviceClient<uav_ros_control::GenerateSearch>("generate_search");

    // Initialize search trajectory caller
    _interceptionTrajectoryClientCaller = nh.serviceClient<uav_ros_control::GenerateInterception>("generate_interception");

}

~PursuitStateMachine()
{}

void uavDistCb(const std_msgs::Float32ConstPtr& msg)
{
    _relativeUAVDistance = msg->data;
}

void ballDistCb(const std_msgs::Float32ConstPtr& msg)
{
    _relativeBALLDistance = msg->data;
}

void uavHeightCb(const std_msgs::Float32ConstPtr& msg)
{
    // TO DO
    _relativeUAVHeight = msg->data;
}

void ballHeightCb(const std_msgs::Float32ConstPtr& msg)
{
    // TO DO
}
void uavYawCb(const std_msgs::Float32ConstPtr& msg)
{
    // TO DO
    _relativeUAVYaw = msg->data;
}

void ballYawCb(const std_msgs::Float32ConstPtr& msg)
{
    // TO DO
}

void uavConfidentCb(const std_msgs::Bool msg)
{
    // TO DO
    _start_following_uav = msg.data;
    _isDetectionActive = true;
    _time_last_detection_msg = ros::Time::now();
}

void ballConfidentCb(const std_msgs::Bool msg)
{
    // TO DO
}

void uavDistanceConfidentCb(const std_msgs::Bool msg){
    _kf_distance_active = msg.data;
}

void figureEstimatorCb(geometry_msgs::PoseStamped msg){
    _interceptionPoint = msg;
    _interceptionPoint.pose.position.z += _interceptionZOffset;
    _interceptionActivated = true;
}
void currentEstimatedTargetPointCb(geometry_msgs::PointStamped msg){
    _currentEstimatedTargetPoint = msg;
    _searchEstimated = true;

}

void toppStatusCb(std_msgs::Bool msg){
    _toppStatus = msg;
}


bool pursuitServiceCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    if (!request.data) // TODO: Handle case when pursuit should not turn on
    {
        ROS_FATAL("PursuitSM::pursuitServiceCb - Pursuit is deactivated.");
        turnOffVisualServo();
        _pursuitActivated = false;
        response.success = false;
        response.message = "Pursuit and visual servo deactivated";
        _searchTrajectoryFlag.data = false;
        return true;
    }

    // Check if pursuit is already activated.
    if (_pursuitActivated)
    {
        ROS_FATAL("PursuitSM::pursuitServiceCb - Pursuit is already active.");
        response.success = true;
        response.message = "Pursuit is already active";
        return true;
    }

    // TO DO
    response.success = true;
    response.message = "Pursuit activated.";
    _pursuitActivated = true;

    return true;
}

void pursuitParamCb(pursuit_param_t& configMsg,uint32_t level)
{
    ROS_WARN("PursuitStateMachine::pursuitParamCb()");
    // TODO: Change internal parameter here from dyn_reconf configMsg
    _uav_distance_offset = configMsg.UAV_distance_offset;
    _ball_distance_offset = configMsg.BALL_distance_offset;
    _uav_z_offset = configMsg.UAV_z_offset;
    _arena_x_size = configMsg.arena_x_size;
    _arena_y_size = configMsg.arena_y_size;
    _arena_x_offset = configMsg.arena_x_offset;
    _arena_y_offset = configMsg.arena_y_offset;
    _arena_yaw_offset = configMsg.arena_yaw_offset;
    _x_takeOff = configMsg.x_takeOff;
    _y_takeOff = configMsg.y_takeOff;
    _search_height = configMsg.desired_height;
    _interceptionZOffset = configMsg.interception_z_offset;
    _yawDeadZoneThreshold = configMsg.yaw_deadzone;
}

void setPursuitParameters(pursuit_param_t& config)
{   
    // TODO: Set initial reconfigure paramters here
    config.UAV_distance_offset = _uav_distance_offset;
    config.BALL_distance_offset = _ball_distance_offset;
    config.UAV_z_offset = _uav_z_offset;
    config.arena_x_size = _arena_x_size;
    config.arena_y_size = _arena_y_size;
    config.arena_x_offset = _arena_x_offset;
    config.arena_y_offset = _arena_y_offset;
    config.arena_yaw_offset = _arena_yaw_offset;
    config.x_takeOff = _x_takeOff;
    config.y_takeOff = _y_takeOff;
    config.desired_height = _search_height;
    config.interception_z_offset = _interceptionZOffset;
    config.yaw_deadzone = _yawDeadZoneThreshold;
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("PursuitStateMachine::initializeParameters()");
    bool initialized = nh.getParam(PARAM_RATE, _rate)
    && nh.getParam(PARAM_UAV_DISTANCE_OFFSET, _uav_distance_offset)
    && nh.getParam(PARAM_BALL_DISTANCE_OFFSET, _ball_distance_offset)
    && nh.getParam(PARAM_ARENA_X_SIZE, _arena_x_size)
    && nh.getParam(PARAM_ARENA_Y_SIZE, _arena_y_size)
    && nh.getParam(PARAM_ARENA_X_OFFSET, _arena_x_offset)
    && nh.getParam(PARAM_ARENA_Y_OFFSET, _arena_y_offset)
    && nh.getParam(PARAM_ARENA_YAW_OFFSET, _arena_yaw_offset)
    && nh.getParam(PARAM_X_TAKEOFF, _x_takeOff)
    && nh.getParam(PARAM_Y_TAKEOFF, _y_takeOff)
    && nh.getParam(PARAM_SEARCH_HEIGHT, _search_height)
    && nh.getParam(PARAM_INTERCEPTION_Z_OFFSET, _interceptionZOffset)
    && nh.getParam(PARAM_YAW_ERROR_DEADZONE, _yawDeadZoneThreshold);
    // TODO: Load all the yaml parameters here 
    // Tip: Define parameter name at the top of the file

    _toppStatus.data = false;
    _inferenceEnabled.data = true;


    ROS_INFO("Node rate: %.2f", _rate);
    if (!initialized)
	{
		ROS_FATAL("PursuitStateMachine::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("PursuitStateMachine parameters not properly initialized.");
	}
}

void requestSearchTrajectory(){
    ROS_INFO("PursuitSM::update status - requesting search trajectory.");
    uav_ros_control::GenerateSearch srv;

    if (_searchEstimated){
        ROS_FATAL("PursuitSM::updateStatus - ESTIMATED saerch requested.");
        srv.request.type = "estimated";
    }
    else{
        ROS_FATAL("PursuitSM::updateStatus - SCAN saerch requested.");
        srv.request.type = "scan";
    }
    srv.request.desired_height = _search_height;
    srv.request.x_size = _arena_x_size;
    srv.request.y_size = _arena_y_size;
    srv.request.x_offset = _arena_x_offset;
    srv.request.y_offset = _arena_y_offset;
    srv.request.yaw_offset = _arena_yaw_offset;
    srv.request.x_takeOff = _x_takeOff;
    srv.request.y_takeOff = _y_takeOff;
    srv.request.estimated_point = _currentEstimatedTargetPoint;

    if(!_searchTrajectoryClientCaller.call(srv)){
        ROS_FATAL("PursuitSM::updateStatus - search trajectory not generated.");
        _currentState = PursuitState::OFF;
        return;
    }

    if (srv.response.success){
        ROS_INFO("PursuitSM::updateStatus - search trajectory successfully generated.");
    }
}

void requestInterceptionTrajectory(){
    ROS_INFO("PursuitSM::update status - requesting interception trajectory.");
    uav_ros_control::GenerateInterception srv;
    srv.request.interception_point = _interceptionPoint;

    if(!_interceptionTrajectoryClientCaller.call(srv)){
        ROS_FATAL("PursuitSM::updateStatus - interception trajectory not generated.");
        ROS_INFO("PursuitSM::updateStatus - OFF state activated.");
        _currentState = PursuitState::OFF;
        return;
    }

    if (srv.response.success){
        ROS_INFO("PursuitSM::updateStatus - interception trajectory successfully generated.");
    }

}

void turnOnVisualServo(){
    // Attempt to turn on visual servo
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = true; 
    ROS_INFO(" Someone requested Visual Servo turn ON.");
    if (!_vsClienCaller.call(req, resp))
    {
        ROS_FATAL("PursuitSM::updateStatus - calling visual servo failed.");
        _currentState = PursuitState::OFF;
        _pursuitActivated = false;
        return;
    }

    if (resp.success)
    {   // Visual servo successfully activated
        ROS_INFO("PursuitSM::updateStatus - Visual servo pursuit is activated.");
        _pursuitActivated = true;
        return;
    }
}

void turnOffVisualServo()
{
    // Attempt to turn off visual servo
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = false;
    if (!_vsClienCaller.call(req, resp))
    {
        ROS_FATAL("PursuitSM::updateStatus - calling visual servo failed.");
        return;
    }

    if (!resp.success)
    {
        ROS_INFO("PursuitSM::updateStatus - visual servo successfully deactivated");
        // Visual servo successfully deactivated
        _currentState = PursuitState::OFF;
        //_pursuitActivated = false; 
        return;
    }
    else
    {
        // Visual servo is still active here...
        ROS_FATAL("PursuitSM::updateStatus - Unable to deactivate visual servo.");
    }
}

void updateState()
{
    // TODO: DO state machine logic here
    checkDetection();

    // 1
    // If visual servo is inactive, deactivate state machine
    // Visual servo can be inactive in state OFF and SEARCH.
    if(_currentState == PursuitState::OFF && _lastState == PursuitState::UAV_FOLLOWING){
        ROS_WARN("PursuitSM::updateStatus - Sleep for 5 secs.");
        ros::Duration(5.0).sleep();
        ros::spinOnce();
    }
    _lastState = _currentState;
    if (_currentState != PursuitState::OFF && !_pursuitActivated && !_interceptionActivated)
    {
        ROS_WARN("PursuitSM::updateStatus - Visual servo is inactive.");
        _currentState = PursuitState::OFF;
        turnOffVisualServo();
        _searchTrajectoryFlag.data = false;
        _inferenceEnabled.data = true;
        ROS_WARN("PursuitSM::updateStatus - OFF State activated.");
        return;
    }
    // Request search trajectory
    if (_currentState == PursuitState::OFF && (!_start_following_uav | !_isDetectionActive) && _pursuitActivated){
        ROS_WARN("PursuitSM::updateStatus - SEARCH state activated.");
        turnOffVisualServo();
        _currentState = PursuitState::SEARCH;
        _searchTrajectoryFlag.data = true;
        _inferenceEnabled.data = true;
        requestSearchTrajectory();
        return;
    }
    // Activate Pursuit algorithm when detection is confident.
    if ((_currentState == PursuitState::OFF | _currentState == PursuitState::SEARCH ) && _start_following_uav && _isDetectionActive && _pursuitActivated)
    {
        ROS_INFO("PursuitSM::updateStatus - Starting visual servo for UAV following.");
        _currentState = PursuitState::UAV_FOLLOWING;
        _searchTrajectoryFlag.data = false;
        _inferenceEnabled.data = true;
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
        _inferenceEnabled.data = true;
        //requestSearchTrajectory();
        ROS_WARN("PursuitSM::updateStatus - OFF State activated.");
        return;
    }

    if (_currentState == PursuitState::UAV_FOLLOWING && _interceptionActivated){
        ROS_WARN("PursuitSM::updateStatus - INTERCEPT state activated.");
        turnOffVisualServo();
        _currentState = PursuitState::INTERCEPTION;
        // generate trajectory
        requestInterceptionTrajectory();
        _searchTrajectoryFlag.data = true;
        _inferenceEnabled.data = true;
        _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
        ros::Duration(2.0).sleep();

        return;
    }

    if (_currentState == PursuitState::INTERCEPTION && !_toppStatus.data && _interceptionActivated){
        ROS_FATAL("PursuitSM::updateStatus - Interception point reached. BALL_GRASPING state activated.");
        _currentState = PursuitState::BALL_GRASPING;
        _searchTrajectoryFlag.data = false;
        ROS_WARN("PursuitSM::updateStatus - CNN inference disabled.");
        _inferenceEnabled.data = false;


        return;
    }

    if (_currentState == PursuitState::UAV_FOLLOWING && _pursuitActivated)
    {
        // Comment this out for testing pursposes
        //_currHeightReference = 0;
        //_currDistanceReference = _uav_distance_offset;

        _inferenceEnabled.data = true;

        _currDistanceReference = _relativeUAVDistance;

        _currHeightReference = _relativeUAVHeight;
        _currYawReference = _relativeUAVYaw;

        if (isRelativeDistanceNan()){
            _currDistanceReference = _maxDistanceReference;
            ROS_WARN("PursuitSM::updateStatus - UAV distance is Nan.");
        }
        else if (!isRelativeDistancePositive()) {
            // pass
            ROS_FATAL("PursuitSM::updateStatus - UAV distance is negative. %f", _relativeUAVDistance);
            _currDistanceReference = _uav_distance_offset;
        }

        return;
    }
}   

bool isRelativeDistanceNan()
{
    return (isnan(_relativeUAVDistance));
}

bool isRelativeDistancePositive()
{
    return (_relativeUAVDistance > 0);
}

void checkDetection(){
    double dt = (ros::Time::now() - _time_last_detection_msg).toSec();
    if (dt > 0.5)
        _isDetectionActive = false;
}

void publishVisualServoSetpoint(double dt)
{
    switch (_currentState)
    {
        // case PursuitState::OFF :
        //     _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
        //     _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
        //     _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
        //     _currVisualServoFeed.yaw = util::calculateYaw(
        //         _currOdom.pose.pose.orientation.x,
        //         _currOdom.pose.pose.orientation.y,
        //         _currOdom.pose.pose.orientation.z,
        //         _currOdom.pose.pose.orientation.w);
        //     break;
        
        case PursuitState::UAV_FOLLOWING : 
            _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
            _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
            _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
            _currVisualServoFeed.yaw = util::calculateYaw(
                _currOdom.pose.pose.orientation.x,
                _currOdom.pose.pose.orientation.y,
                _currOdom.pose.pose.orientation.z,
                _currOdom.pose.pose.orientation.w);
            break;
    }

    _currVisualServoFeed.header.stamp = ros::Time::now();
    _pubVisualServoFeed.publish(_currVisualServoFeed);

    // Publish currrent state
    std_msgs::Int32 stateMsg;
    stateMsg.data = _currentState;
    _pubVssmState.publish(stateMsg);
}

void publishOffsets()
{
    if (_currentState == PursuitState::UAV_FOLLOWING){
        std_msgs::Float32 offsetYMsg;
        offsetYMsg.data = -1 * _uav_distance_offset;
        _pubOffsetY.publish(offsetYMsg);

        std_msgs::Float32 offsetZMsg;
        offsetZMsg.data = _uav_z_offset;
        _pubOffsetZ.publish(offsetZMsg);  
    }
    // later for ball
    // if (_currentState == PursuitState::BALL_FOLLOWING){
    //     std_msgs::Float32 offsetXMsg;
    //     offsetXMsg.data = BALL_DISTANCE_OFFSET; // TODO: Put some value here 
    //     _pubOffsetY.publish(offsetXMsg);  
    // }

}

void publishErrors(){
    if (_currentState == PursuitState::UAV_FOLLOWING){
        std_msgs::Float32 msg;

        msg.data = -1 * _currDistanceReference;
        _pubYError.publish(msg);

        msg.data = _currHeightReference;
        _pubZError.publish(msg);

        // Deadzone for error
        if (abs(_currYawReference) < _yawDeadZoneThreshold){
            _currYawReference = 0;
        }
        else{
            _currYawReference -= _yawDeadZoneThreshold; 
        }

        msg.data = _currYawReference;
        _pubYawError.publish(msg);
    }
    // later
    // if (_currentState == PursuitState::BALL_FOLLOWING){
    //     std_msgs::Float32 msg;

    //     msg.data = _relativeBALLDistance;
    //     _pubXError.publish(msg);

    //     msg.data = _relativeBALLHeight;
    //     _pubZError.publish(msg);

    //     msg.data = _relativeBALLYaw;
    //     _pubYawError.publish(msg);
    // }
}
void odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    _currOdom = *msg;
}

void run()
{
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;
	while (ros::ok())
	{
		ros::spinOnce();
        checkDetection();
        updateState();
        _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
        _pubInferenceEnabled.publish(_inferenceEnabled);
        publishOffsets();
        publishErrors();
        publishVisualServoSetpoint(dt);
        loopRate.sleep();
    }
}

private:

    double _rate = 50;

    /* Service Pursuit */
	ros::ServiceServer _servicePursuit;
    bool _pursuitActivated = false;
    PursuitState _currentState = PursuitState::OFF, _lastState = PursuitState::OFF;
    
    /* Client for calling visual servo and search trajectory */
    ros::ServiceClient _vsClienCaller, _searchTrajectoryClientCaller, _interceptionTrajectoryClientCaller;

    /* Offset subscriber and publisher */
    ros::Publisher _pubVssmState, _pubOffsetY, _pubOffsetZ;

    /* Trajectory */
    ros::Publisher _pubSearchTrajectoryFlag;
    ros::Subscriber _subToppStatus;
    std_msgs::Bool _searchTrajectoryFlag, _toppStatus;

    /* Error publishers */
    ros::Publisher _pubXError, _pubYError, _pubZError, _pubYawError;

    /* CNN inference flag publisher. */
    ros::Publisher _pubInferenceEnabled;
    std_msgs::Bool _inferenceEnabled;

    /* Error subscribers */
    ros::Subscriber _subUAVDist, _subBALLDist;
    ros::Subscriber _subUAVHeightError, _subBALLHeightError;
    ros::Subscriber _subUAVYawError, _subBALLYawError;
    double _relativeUAVDistance = INVALID_DISTANCE, _relativeBALLDistance = INVALID_DISTANCE;
    double _relativeUAVHeight,_relativeUAVYaw = 0;

    /* Confidence subscribers */
    ros::Subscriber _subUAVPursuitConfident, _subBALLPursuitConfident;
    ros::Subscriber _subUAVDistanceConfident;

    /* Interception subscribers*/
    ros::Subscriber _subInterceptionPoint, _subCurrentEstimatedTargetPoint;
    float _interceptionZOffset;
    bool _searchEstimated = false;
    geometry_msgs::PointStamped _currentEstimatedTargetPoint;

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
    float _uav_z_offset, _yawDeadZoneThreshold;
    ros::Time _time_last_detection_msg;
    float _maxDistanceReference = 15.0;
    float _arena_x_size, _arena_y_size, _arena_x_offset, _arena_y_offset, _arena_yaw_offset;
    double _x_takeOff, _y_takeOff, _search_height;
    // Interception
    geometry_msgs::PoseStamped _interceptionPoint;
    bool _interceptionActivated = false;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _pursuitConfigMutex;
    dynamic_reconfigure::Server<pursuit_param_t>
        _pursuitConfigServer {_pursuitConfigMutex, ros::NodeHandle(PURSUIT_DYN_RECONF)};
    dynamic_reconfigure::Server<pursuit_param_t>::CallbackType _pursuitParamCallback;
};
}

#endif /* VISUAL_SERVO_PURSUIT_STATE_MACHINE_H*/

