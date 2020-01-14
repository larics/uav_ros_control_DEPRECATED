#ifndef VISUAL_SERVO_STATE_MACHINE_H
#define VISUAL_SERVO_STATE_MACHINE_H

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
#include <uav_ros_control/VisualServoStateMachineParametersConfig.h>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/filters/NonlinearFilters.h>

namespace uav_reference 
{

typedef uav_ros_control::VisualServoStateMachineParametersConfig vssm_param_t;
#define VSSM_DYN_RECONF             "vs_state_machine"
#define PARAM_MIN_ERROR             "visual_servo/state_machine/min_error"
#define PARAM_MIN_YAW_ERROR         "visual_servo/state_machine/min_yaw_error"
#define PARAM_TOUCHDOWN_HEIGHT      "visual_servo/state_machine/touchdown_height"
#define PARAM_TOUCHDOWN_DELTA       "visual_servo/state_machine/touchdown_delta"
#define PARAM_TOUCHDOWN_DURATION    "visual_servo/state_machine/touchdown_duration"
#define PARAM_RATE                  "visual_servo/state_machine/rate"
#define PARAM_DESCENT_SPEED         "visual_servo/state_machine/descent_speed"
#define PARAM_ASCENT_SPEED         "visual_servo/state_machine/ascent_speed"
#define PARAM_DET_COUNTER           "visual_servo/state_machine/detection_counter"
#define PARAM_AFTER_TD_HEIGHT       "visual_servo/state_machine/after_touchdown_height"
#define INVALID_DISTANCE -1

enum VisualServoState {
    OFF,
    BRICK_ALIGNMENT,
    DESCENT,
    TOUCHDOWN
};

class VisualServoStateMachine
{

public:

VisualServoStateMachine(ros::NodeHandle& nh)
{   
	initializeParameters(nh);
    // Define Publishers
    _pubVisualServoFeed = 
        nh.advertise<uav_ros_control_msgs::VisualServoProcessValues>("visual_servo/process_value", 1);
    _pubVssmState = nh.advertise<std_msgs::Int32>("visual_servo_sm/state", 1);

    // Define Subscribers
    _subOdom =
        nh.subscribe("odometry", 1, &uav_reference::VisualServoStateMachine::odomCb, this);
    // Note from past Lovro : This is changed to debug/yaw_error because that holds actual information about the yaw_error
    // TODO: Change this to make more sense
    _subYawError = 
        nh.subscribe("debug/yaw_error", 1, &uav_reference::VisualServoStateMachine::yawErrorCb, this); 
    _subNContours =
        nh.subscribe("n_contours", 1, &uav_reference::VisualServoStateMachine::nContoursCb, this);
    _subPatchCentroid =
        nh.subscribe("centroid_point", 1, &uav_reference::VisualServoStateMachine::patchCentroidCb, this);

    // Setup dynamic reconfigure server
	vssm_param_t  vssmConfig;
	setVSSMParameters(vssmConfig);
	_vssmConfigServer.updateConfig(vssmConfig);
	_vssmParamCallback = boost::bind(
		&uav_reference::VisualServoStateMachine::vssmParamCb, this, _1, _2);
	_vssmConfigServer.setCallback(_vssmParamCallback);

    // Setup brick pickup service callback
    _serviceBrickPickup = nh.advertiseService(
			"brick_pickup",
			&uav_reference::VisualServoStateMachine::brickPickupServiceCb,
			this);

    // Initialize visual servo client caller
    _vsClienCaller = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("visual_servo");
}

~VisualServoStateMachine()
{}

void nContoursCb(const std_msgs::Int32ConstPtr& msg)
{
    _nContours = msg->data;
    if (msg->data == 0 && _currentState != VisualServoState::TOUCHDOWN && 
        _currentState !=VisualServoState::OFF)
    {
        turnOffVisualServo();
    }
}

void patchCentroidCb(const geometry_msgs::PointStamped& msg)
{
    _relativeBrickDistance = msg.point.z;
    _currPosError = sqrt( pow(msg.point.x, 2) + pow(msg.point.y, 2));
}

bool brickPickupServiceCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    if (!request.data || _nContours == 0)
    {
        if (!request.data)
            ROS_FATAL("VSSM::brickPickupServiceCb - brick pickup deactivation requested.");
        else if (_nContours == 0)
            ROS_FATAL("VSSM::brickPickupServiceCb - no contours found.");

        turnOffVisualServo();
        _brickPickupActivated = false;
        response.success = false;
        response.message = "Visual servo and brick pickup deactivated";
        return true;
    }

    // Check if brick pickup is already activated.
    if (_brickPickupActivated)
    {
        ROS_FATAL("VSSM::brickPickupServiceCb - brick pickup is already active.");
        response.success = false;
        response.message = "Brick pickup is already active";
        return true;
    }

    // Try calling visual servo
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = true;
    if (!_vsClienCaller.call(req, resp))
    {
        ROS_FATAL("VSSM::brickPickupServiceCb - calling visual servo failed.");
        response.success = false;
        response.message = "Service caller for visual servo failed.";
        _currentState = VisualServoState::OFF;
        return true;
    }

    if (resp.success)
    {
        // Visual servo successfully activated
        ROS_INFO("VSSM::brickPickupServiceCb() - brick pickup activated.");
        response.success = true;
        response.message = "Visual servo enabled - brick pickup activated.";
        _brickPickupActivated = true;
        return true;
    }
    
    ROS_WARN("VSSM::brickPickupServiceCb - unable to activate brick pickup.");
    response.success = false;
    response.message = "Visual servo failed to start - brick pickup inactive.";
    _brickPickupActivated = false;

    return true;
}

void vssmParamCb(vssm_param_t& configMsg,uint32_t level)
{
    ROS_WARN("VisualServoStateMachine::vssmParamCb()");
    _minTargetError = configMsg.min_error;
    _minYawError = configMsg.min_yaw_error;
    _touchdownHeight = configMsg.touchdown_height;
    _touchdownDelta = configMsg.touchdown_delta;
    _touchdownDuration = configMsg.touchdown_duration;
    _descentSpeed = configMsg.descent_speed;
    _afterTouchdownHeight = configMsg.after_touchdown_height;
    _descentCounterMax = configMsg.detection_counter;
    _ascentSpeed = configMsg.ascent_speed;
}

void setVSSMParameters(vssm_param_t& config)
{
    config.min_error = _minTargetError;
    config.min_yaw_error = _minYawError;
    config.touchdown_delta = _touchdownDelta;
    config.touchdown_duration = _touchdownDuration;
    config.touchdown_height = _touchdownHeight;
    config.descent_speed = _descentSpeed;
    config.after_touchdown_height = _afterTouchdownHeight;
    config.detection_counter = _descentCounterMax;
    config.ascent_speed = _ascentSpeed;
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("VisualServoStateMachine::initializeParameters()");
    bool initialized = nh.getParam(PARAM_MIN_YAW_ERROR, _minYawError)
        && nh.getParam(PARAM_MIN_ERROR, _minTargetError)
		&& nh.getParam(PARAM_TOUCHDOWN_HEIGHT, _touchdownHeight)
		&& nh.getParam(PARAM_TOUCHDOWN_DURATION, _touchdownDuration)
        && nh.getParam(PARAM_TOUCHDOWN_DELTA, _touchdownDelta)
        && nh.getParam(PARAM_DESCENT_SPEED, _descentSpeed)
        && nh.getParam(PARAM_ASCENT_SPEED, _ascentSpeed)
        && nh.getParam(PARAM_RATE, _rate)
        && nh.getParam(PARAM_AFTER_TD_HEIGHT, _afterTouchdownHeight)
        && nh.getParam(PARAM_DET_COUNTER, _descentCounterMax);

    ROS_INFO("Node rate: %.2f", _rate);
    ROS_INFO("Minimum yaw error: %.2f", _minYawError);
    ROS_INFO("Descent speed: %.2f", _descentSpeed);
    ROS_INFO("Touchdown height: %.2f", _touchdownHeight);
    ROS_INFO("Touchdown duration: %.2f", _touchdownDuration);
    ROS_INFO("Touchdown delta: %.2f", _touchdownDelta);
    ROS_INFO("After touchdown height: %.2f", _afterTouchdownHeight);
    ROS_INFO("Detection counter: %d", _descentTransitionCounter);
    if (!initialized)
	{
		ROS_FATAL("VisualServoStateMachine::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("VisualServoStateMachine parameters not properly initialized.");
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
        ROS_FATAL("VSSM::updateStatus - calling visual servo failed.");
        return;
    }

    if (!resp.success)
    {
        ROS_INFO("VSSM::updateStatus - visual servo successfully deactivated");
        // Visual servo successfully activated
        _currentState = VisualServoState::OFF;
        ROS_INFO("VSSM::updateStatus - OFF state activated. ");
        _brickPickupActivated = false; 
        ROS_INFO("VSSM::updateStatus - Brick pickup finished.");
        return;
    }
    else
    {
        // Visual servo is still active here...
        ROS_FATAL("VSSM::updateStatus - Touchdown finished but unable to deactivate visual servo.");
    }
}

void updateState()
{
    // If visual servo is inactive, deactivate state machine
    if (_currentState != VisualServoState::OFF && !_brickPickupActivated)
    {
        ROS_WARN("VSSM::updateStatus - Visual servo is inactive.");
        _currentState = VisualServoState::OFF;
        _brickPickupActivated = false;
        turnOffVisualServo();
        ROS_WARN("VSSM::updateStatus - OFF State activated.");
        return;
    }

    // If brick pickup is activate start brick alignment first
    if (_currentState == VisualServoState::OFF && _brickPickupActivated)
    {
        ROS_INFO("VSSM::updateStatus - Brick pickup requested");
        _currHeightReference = _currOdom.pose.pose.position.z;
        _descentTransitionCounter = 0;
        _currentState = VisualServoState::BRICK_ALIGNMENT;
        ROS_INFO("VSSM::updateStatus - BRICK_ALIGNMENT state activated with height: %2f.", _currHeightReference);
        return;
    }

    // Update the transition counter
    if (_currentState == VisualServoState::BRICK_ALIGNMENT &&
        _currPosError < _minTargetError &&
        fabs(_currYawError) < _minYawError) {
        _descentTransitionCounter++;
    }

    // If brick alignemnt is activated and target error is withing range start descent
    if (_currentState == VisualServoState::BRICK_ALIGNMENT &&
        _descentTransitionCounter > _descentCounterMax)
    {
        _currentState = VisualServoState::DESCENT;
        ROS_INFO("VSSM::updateStatus - DESCENT state activated");
        return;
    }

    // if height is below touchdown treshold start touchdown
    if (_currentState == VisualServoState::DESCENT &&
        isRelativeDistanceValid() && 
        _relativeBrickDistance <= _touchdownHeight)
    {
        _currentState = VisualServoState::TOUCHDOWN;
        _touchdownTime = 0;
        //_currHeightReference = _relativeBrickDistance;
        ROS_INFO("VSSM::UpdateStatus - TOUCHDOWN state activated");
        return;
    }

    // If touchdown time is exceeded, touchdown state is considered finished
    if (_currentState == VisualServoState::TOUCHDOWN &&
        _touchdownTime >= _touchdownDuration && 
        _currHeightReference >= _afterTouchdownHeight) 
    {
        ROS_INFO("VSSM::updateStatus - Touchdown duration finished.");
        turnOffVisualServo();
    }
}   

bool isRelativeDistanceValid()
{
    return _relativeBrickDistance > 0;
}

void publishVisualServoSetpoint(double dt)
{
    // Set VisualServoFeed to odometry
    _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
    _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
    _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
    _currVisualServoFeed.yaw = util::calculateYaw(
        _currOdom.pose.pose.orientation.x,
        _currOdom.pose.pose.orientation.y,
        _currOdom.pose.pose.orientation.z,
        _currOdom.pose.pose.orientation.w);
        
    switch (_currentState)
    {
        case VisualServoState::OFF :
            // pass
            break;
        
        case VisualServoState::BRICK_ALIGNMENT : 
            _currVisualServoFeed.z = _currHeightReference;
            break;
        
        case VisualServoState::DESCENT : 
            _currVisualServoFeed.z = _currHeightReference - _descentSpeed * dt;
            _currHeightReference = _currVisualServoFeed.z;
            _currVisualServoFeed.yaw = 0;
            break;
        
        case VisualServoState::TOUCHDOWN : 
            _currVisualServoFeed.x = 0;
            _currVisualServoFeed.y = 0;
            double dz = 2 * _touchdownDelta / _touchdownDuration * dt;
            if (_touchdownTime < _touchdownDuration/2.0)
                _currVisualServoFeed.z = _currHeightReference - dz;
            else
                _currVisualServoFeed.z = _currHeightReference + _ascentSpeed * dt;
            _currHeightReference = _currVisualServoFeed.z;
            _currVisualServoFeed.yaw = 0;
            _touchdownTime +=dt;
            break;
    }

    _currVisualServoFeed.header.stamp = ros::Time::now();
    _pubVisualServoFeed.publish(_currVisualServoFeed);

    // Publish currrent state
    std_msgs::Int32 stateMsg;
    stateMsg.data = _currentState;
    _pubVssmState.publish(stateMsg);
}

void odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    _currOdom = *msg;
}

void yawErrorCb(const std_msgs::Float32ConstPtr& msg)
{
    _currYawError = msg->data;
}

void run()
{
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;
	while (ros::ok())
	{
		ros::spinOnce();
        updateState();
        publishVisualServoSetpoint(dt);

        if (_currentState == VisualServoState::DESCENT && !isRelativeDistanceValid())
            ROS_FATAL("*** FATAL - BLIND DESCENT ***"); // TODO: :) (?)

        loopRate.sleep();
    }
}

private:

    double _rate = 50;

    /* Service brick pickup */
	ros::ServiceServer _serviceBrickPickup;
    bool _brickPickupActivated = false;
    VisualServoState _currentState = VisualServoState::OFF;
    
    /* Client for calling visual servo */
    ros::ServiceClient _vsClienCaller;

    /* Offset subscriber and publisher */
    ros::Publisher _pubVssmState;

    /* Pose publisher */
    ros::Publisher _pubVisualServoFeed;
    uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;
    nav_msgs::Odometry _currOdom;

    /* Yaw error subscriber */
    ros::Subscriber _subYawError;
    double _currYawError = 1e5, _currPosError = 1e5;
    double _minYawError, _minTargetError;

    ros::Subscriber _subBrickDist;
    ros::Subscriber _subPatchCentroid;
    double _relativeBrickDistance = INVALID_DISTANCE;

    /* Contour subscriber */
    ros::Subscriber _subNContours;
    int _nContours;

    /* Touchdown mode parameters */
    double _touchdownHeight, _touchdownDelta, _touchdownDuration, _touchdownTime, _descentCounterMax;
    double _currHeightReference, _descentSpeed, _ascentSpeed, _afterTouchdownHeight; 
    int _descentTransitionCounter = 0;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _vssmConfigMutex;
    dynamic_reconfigure::Server<vssm_param_t>
        _vssmConfigServer {_vssmConfigMutex, ros::NodeHandle(VSSM_DYN_RECONF)};
    dynamic_reconfigure::Server<vssm_param_t>::CallbackType _vssmParamCallback;
};
}

#endif /* VISUAL_SERVO_STATE_MACHINE_H */