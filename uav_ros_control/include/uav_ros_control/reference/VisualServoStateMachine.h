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
#include <uav_ros_control/reference/PickupStates.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

namespace uav_reference 
{

typedef uav_ros_control::VisualServoStateMachineParametersConfig vssm_param_t;
#define VSSM_DYN_RECONF             "brick_config/vs_state_machine"
#define PARAM_MIN_ERROR             "visual_servo/state_machine/min_error"
#define PARAM_MIN_TD_TAR_ERROR_Z      "visual_servo/state_machine/min_touchdown_target_position_error_z"
#define PARAM_MIN_TD_UAV_VEL_ERROR_Z  "visual_servo/state_machine/min_touchdown_uav_velocity_error_z"
#define PARAM_MIN_TD_TAR_ERROR_XY      "visual_servo/state_machine/min_touchdown_target_position_error_xy"
#define PARAM_MIN_TD_UAV_VEL_ERROR_XY  "visual_servo/state_machine/min_touchdown_uav_velocity_error_xy"
#define PARAM_MIN_TD_ALIGN_DURATION "visual_servo/state_machine/min_touchdown_align_duration"
#define PARAM_MIN_YAW_ERROR         "visual_servo/state_machine/min_yaw_error"
#define PARAM_VS_HEIGHT_DISABLE     "visual_servo/state_machine/disable_visual_servo_touchdown_height"
#define PARAM_TOUCHDOWN_HEIGHT      "visual_servo/state_machine/touchdown_height"
#define PARAM_MAGNET_OFFSET         "visual_servo/state_machine/magnet_offset"
#define PARAM_TOUCHDOWN_SPEED       "visual_servo/state_machine/touchdown_speed"
#define PARAM_RATE                  "visual_servo/state_machine/rate"
#define PARAM_DESCENT_SPEED         "visual_servo/state_machine/descent_speed"
#define PARAM_ASCENT_SPEED         "visual_servo/state_machine/ascent_speed"
#define PARAM_DET_COUNTER           "visual_servo/state_machine/detection_counter"
#define PARAM_AFTER_TD_HEIGHT       "visual_servo/state_machine/after_touchdown_height"
#define PARAM_BRICK_ALIGN_HEIGHT    "visual_servo/state_machine/brick_alignment_height"
#define INVALID_DISTANCE -1

using namespace pickup_states;
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
    _pubVelError = nh.advertise<geometry_msgs::Vector3>("visual_servo_sm/velocity_error", 1);
    _pubTargetError = nh.advertise<geometry_msgs::Vector3>("visual_servo_sm/pos_error", 1);
    _pubVisualServoSuccess = nh.advertise<std_msgs::Bool>("visual_servo_sm/success", 1);

    // Define Subscribers
    _subOdom =
        nh.subscribe("odometry", 1, &uav_reference::VisualServoStateMachine::odomCb, this);
    _subNContours =
        nh.subscribe("n_contours", 1, &uav_reference::VisualServoStateMachine::nContoursCb, this);
    _subPatchCentroid_local = 
        nh.subscribe("centroid_point", 1, &uav_reference::VisualServoStateMachine::localCentroidPointCb, this);
    _subTrajectory = 
        nh.subscribe("carrot/trajectory", 1, &uav_reference::VisualServoStateMachine::trajCb, this);

    // Setup dynamic reconfigure server
	vssm_param_t  vssmConfig;
	setVSSMParameters(vssmConfig);
	_vssmConfigServer.updateConfig(vssmConfig);
	_vssmParamCallback = boost::bind(
		&uav_reference::VisualServoStateMachine::vssmParamCb, this, _1, _2);
	_vssmConfigServer.setCallback(_vssmParamCallback);

    // Setup baloon pop service callback
    _serviceBrickPickup = nh.advertiseService(
			"pop_baloon",
			&uav_reference::VisualServoStateMachine::popBaloonServiceCb,
			this);

    // Initialize visual servo client caller
    _vsClienCaller = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("visual_servo");
}

~VisualServoStateMachine()
{}

void trajCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint& point)
{
    _trajPoint = point;
}

void nContoursCb(const std_msgs::Int32ConstPtr& msg)
{
    _nContours = msg->data;
    if (msg->data == 0 && _currentState == BaloonPopState::ALIGNEMNT)
    {
        ROS_FATAL("VSSM - patch count is 0.");
        std_msgs::Bool success;
        success.data = false;
        _pubVisualServoSuccess.publish(success);
        turnOffVisualServo();
    }
    _timeLastContour = ros::Time::now().toSec();
}

void localCentroidPointCb(const geometry_msgs::Vector3& msg) 
{
    _localCentroid = msg;
    if (msg.x == INVALID_DISTANCE) {
        _relativeBaloonDistance = INVALID_DISTANCE;
    } else {
        _relativeBaloonDistance = msg.x;
    }
    _timeLastCentroidLocal = ros::Time::now().toSec();
}

bool healthyNumberOfPublishers() 
{
    ROS_FATAL_COND(!_subNContours.getNumPublishers() > 0, "VSSM - 'n_contours' topic publisher missing");
    ROS_FATAL_COND(!_subOdom.getNumPublishers() > 0, "VSSM - 'odometry' topic publisher missing");
    ROS_FATAL_COND(!_subPatchCentroid_local.getNumPublishers() > 0, "VSSM - 'centroid_local' topic publisher missing");
    
    return _subNContours.getNumPublishers() > 0 
        && _subOdom.getNumPublishers() > 0
        && _subPatchCentroid_local.getNumPublishers() > 0;
}

bool popBaloonServiceCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    if (!request.data || stateMachineDisableConditions() || !healthyNumberOfPublishers() )
    {
        if (!healthyNumberOfPublishers())
            ROS_FATAL_THROTTLE(THROTTLE_TIME, "VSSM::popBaloonServiceCb - check connected publishers.");
        
        if (!request.data)
            ROS_FATAL_THROTTLE(THROTTLE_TIME, "VSSM::popBaloonServiceCb - baloon pop deactivation requested.");
        
        if (_nContours == 0)
            ROS_FATAL_THROTTLE(THROTTLE_TIME, "VSSM::popBaloonServiceCb - no contours found.");

        if (!isRelativeDistanceValid(_relativeBaloonDistance))
            ROS_FATAL_THROTTLE(THROTTLE_TIME, "VSSM::popBaloonServiceCb - distances invalid");

        turnOffVisualServo();
        _baloonPopActivated = false;
        response.success = false;
        response.message = "Visual servo and baloon pop deactivated";
        return true;
    }

    // Check if baloon pop is already activated.
    if (_baloonPopActivated)
    {
        ROS_FATAL("VSSM::popBaloonServiceCb - baloon pop is already active.");
        response.success = false;
        response.message = "baloon pop is already active";
        return true;
    }

    // Try calling visual servo
    // ros::spinOnce();
    // publishVisualServoSetpoint(1.0 / _rate);
    
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = true;
    if (!_vsClienCaller.call(req, resp))
    {
        ROS_FATAL("VSSM::popBaloonServiceCb - calling visual servo failed.");
        response.success = false;
        response.message = "Service caller for visual servo failed.";
        _currentState = BaloonPopState::OFF;
        return true;
    }

    if (resp.success)
    {
        // Visual servo successfully activated
        ROS_INFO("VSSM::popBaloonServiceCb() - baloon pop activated.");
        response.success = true;
        response.message = "Visual servo enabled - baloon pop activated.";
        _baloonPopActivated = true;
        return true;
    }
    
    ROS_WARN("VSSM::popBaloonServiceCb - unable to activate baloon pop.");
    response.success = false;
    response.message = "Visual servo failed to start - baloon pop inactive.";
    _baloonPopActivated = false;

    return true;
}

void vssmParamCb(vssm_param_t& configMsg,uint32_t level)
{
    ROS_WARN("VisualServoStateMachine::vssmParamCb()");
    _minTargetError = configMsg.min_error;
    //_minYawError = configMsg.min_yaw_error;
    _touchdownHeight = configMsg.touchdown_height;
    _magnetOffset = configMsg.magnet_offset;
    _touchdownSpeed = configMsg.touchdown_speed;
    _descentSpeed = configMsg.descent_speed;
    _afterTouchdownHeight = configMsg.after_touchdown_height;
    _descentCounterMax = configMsg.detection_counter;
    _ascentSpeed = configMsg.ascent_speed;
    _minTouchdownTargetPositionError_z = configMsg.min_touchdown_target_position_error_z;
    _minTouchdownUavVelocityError_z = configMsg.min_touchdown_uav_velocity_error_z;
    _minTouchdownTargetPositionError_xy = configMsg.min_touchdown_target_position_error_xy;
    _minTouchdownUavVelocityError_xy = configMsg.min_touchdown_uav_velocity_error_xy;
    _minTouchdownAlignDuration = configMsg.min_touchdown_align_duration;
    _visualServoDisableHeight = configMsg.disable_visual_servo_touchdown_height;
    _brickAlignHeight = configMsg.brick_alignment_height;
}

void setVSSMParameters(vssm_param_t& config)
{
    config.min_error = _minTargetError;
    //config.min_yaw_error = _minYawError;
    config.magnet_offset = _magnetOffset;
    config.touchdown_speed = _touchdownSpeed;
    config.touchdown_height = _touchdownHeight;
    config.descent_speed = _descentSpeed;
    config.after_touchdown_height = _afterTouchdownHeight;
    config.detection_counter = _descentCounterMax;
    config.ascent_speed = _ascentSpeed;
    config.min_touchdown_target_position_error_z = _minTouchdownTargetPositionError_z;
    config.min_touchdown_uav_velocity_error_z = _minTouchdownUavVelocityError_z;
    config.min_touchdown_target_position_error_xy = _minTouchdownTargetPositionError_xy;
    config.min_touchdown_uav_velocity_error_xy = _minTouchdownUavVelocityError_xy;
    config.min_touchdown_align_duration = _minTouchdownAlignDuration;
    config.disable_visual_servo_touchdown_height = _visualServoDisableHeight;
    config.brick_alignment_height = _brickAlignHeight;
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("VisualServoStateMachine::initializeParameters()");
    bool initialized = 
        //nh.getParam(PARAM_MIN_YAW_ERROR, _minYawError)
        nh.getParam(PARAM_MIN_TD_TAR_ERROR_XY, _minTouchdownTargetPositionError_xy)
        && nh.getParam(PARAM_MIN_TD_UAV_VEL_ERROR_XY, _minTouchdownUavVelocityError_xy)
        && nh.getParam(PARAM_MIN_TD_TAR_ERROR_Z, _minTouchdownTargetPositionError_z)
        && nh.getParam(PARAM_MIN_TD_UAV_VEL_ERROR_Z, _minTouchdownUavVelocityError_z)
        && nh.getParam(PARAM_VS_HEIGHT_DISABLE, _visualServoDisableHeight)
        && nh.getParam(PARAM_MIN_TD_ALIGN_DURATION, _minTouchdownAlignDuration)
        && nh.getParam(PARAM_BRICK_ALIGN_HEIGHT, _brickAlignHeight)
        && nh.getParam(PARAM_MIN_ERROR, _minTargetError)
		&& nh.getParam(PARAM_TOUCHDOWN_HEIGHT, _touchdownHeight)
		&& nh.getParam(PARAM_TOUCHDOWN_SPEED, _touchdownSpeed)
        && nh.getParam(PARAM_MAGNET_OFFSET, _magnetOffset)
        && nh.getParam(PARAM_DESCENT_SPEED, _descentSpeed)
        && nh.getParam(PARAM_ASCENT_SPEED, _ascentSpeed)
        && nh.getParam(PARAM_RATE, _rate)
        && nh.getParam(PARAM_AFTER_TD_HEIGHT, _afterTouchdownHeight)
        && nh.getParam(PARAM_DET_COUNTER, _descentCounterMax);

    _afterTouchdownHeight_GPS = _afterTouchdownHeight;
    ROS_INFO("Node rate: %.2f", _rate);
    //ROS_INFO("Minimum yaw error: %.2f", _minYawError);
    ROS_INFO("Brick alignment height: %.2f", _brickAlignHeight);
    ROS_INFO("Min target error %.2f", _minTargetError);
    ROS_INFO("Touchdown position target error [%.2f, %.2f, %.2f]", 
        _minTouchdownTargetPositionError_xy, _minTouchdownTargetPositionError_xy, _minTouchdownTargetPositionError_z);
    ROS_INFO("Touchdown uav velocity error [%.2f, %.2f, %.2f]", 
        _minTouchdownUavVelocityError_xy, _minTouchdownUavVelocityError_xy, _minTouchdownUavVelocityError_z);
    ROS_INFO("Min touchdown alignment duration %.2f", _minTouchdownAlignDuration);
    ROS_INFO("Visual servo disable height %.2f", _visualServoDisableHeight);
    ROS_INFO("Descent speed: %.2f", _descentSpeed);
    ROS_INFO("Touchdown height: %.2f", _touchdownHeight);
    ROS_INFO("Touchdown speed: %.2f", _touchdownSpeed);
    ROS_INFO("Magnet offset: %.2f", _magnetOffset);
    ROS_INFO("After touchdown height: %.2f", _afterTouchdownHeight);
    ROS_INFO("Detection counter: %d", _descentCounterMax);
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
        ROS_INFO_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - visual servo successfully deactivated");
        // Visual servo successfully activated
        _currentState = BaloonPopState::OFF;
        ROS_INFO_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - OFF state activated. ");
        _baloonPopActivated = false; 
        ROS_INFO_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - baloon pop finished.");
        return;
    }
    else
    {
        // Visual servo is still active here...
        ROS_FATAL("VSSM::updateStatus - Touchdown finished but unable to deactivate visual servo.");
    }
}

bool stateMachineDisableConditions()
{
    return !subscribedTopicsActive()
        || !isRelativeDistanceValid(_relativeBaloonDistance)
        || _nContours == 0;
}

void updateState()
{
    if (_currentState != BaloonPopState::OFF && !_baloonPopActivated ||  // If visual servo is 
        _currentState == BaloonPopState::ALIGNEMNT && stateMachineDisableConditions())
    {
        // deactivate state machine
        ROS_WARN_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - Visual servo is inactive.");
        _currentState = BaloonPopState::OFF;
        _baloonPopActivated = false;
        std_msgs::Bool success;
        success.data = false;
        _pubVisualServoSuccess.publish(success);
        turnOffVisualServo();
        ROS_WARN_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - OFF State activated.");
        return;
    }

    // If baloon pop is activated start alignment first
    if (_currentState == BaloonPopState::OFF 
        && _baloonPopActivated
        && isRelativeDistanceValid(_relativeBaloonDistance))
    {
        ROS_INFO("VSSM::updateStatus - baloon pop requested");
        _currDistanceReference = _relativeBaloonDistance;
        _descentTransitionCounter = 0;
        _currentState = BaloonPopState::ALIGNEMNT;
        ros::spinOnce();
        _popXReference = _trajPoint.transforms.front().translation.x;
        _popYReference = _trajPoint.transforms.front().translation.y;
        ROS_INFO("[%.3f, %.3f]", _popXReference, _popYReference);
        ROS_INFO("VSSM::updateStatus - ALIGNMENT state activated with baloon distance: %2f.", _currDistanceReference);
        return;
    }

    if (_currentState == BaloonPopState::ALIGNEMNT && 
        isTargetInThreshold(_minTargetError, _minTargetError, _minTargetError, _brickAlignHeight))
    {
        _descentTransitionCounter++;
        ROS_INFO("VSSM::updateStatus aligned: %d", _descentTransitionCounter);
    }

    // If brick alignemnt is activated and target error is withing range start descent
    if (_currentState == BaloonPopState::ALIGNEMNT &&
        _descentTransitionCounter > 15)
    {
        _currentState = BaloonPopState::POP;
        ROS_INFO("VSSM::updateStatus - ALIGNED! activate POP");
        return;
    }

    // If pop is activated and no contour is found, go to BACK state
    if (_currentState == BaloonPopState::POP && !isRelativeDistanceValid(_relativeBaloonDistance)) {
        _currentState = BaloonPopState::BACK;
        _backStateTime = 0;
        _backStateDuration = 1;
        ROS_INFO("VSSM::updateStatus - baloon is popped, activate BACK");
        return;
    }

    // If BACK is activated and back time is elapsed, go to off
    if (_currentState == BaloonPopState::BACK && _backStateTime > _backStateDuration) {
        _currentState == BaloonPopState::OFF;
        ROS_INFO("VSSM::updateStatus - BACK is finished, turning off visual servo");
        std_msgs::Bool success;
        success.data = true;
        _pubVisualServoSuccess.publish(success);
        turnOffVisualServo();
    }
}   

bool isUavVelcityInThreshold()
{
    double velx = fabs(_currOdom.twist.twist.linear.x),
        vely = fabs(_currOdom.twist.twist.linear.y),
        velz = fabs(_currOdom.twist.twist.linear.z);

    geometry_msgs::Vector3 msg;
    msg.x = velx;
    msg.y = vely;
    msg.z = velz;
    _pubVelError.publish(msg);

    return velx < _minTouchdownUavVelocityError_xy
        && vely < _minTouchdownUavVelocityError_xy
        && velz < _minTouchdownUavVelocityError_z;
}

bool isTargetInThreshold(const double minX, const double minY, const double minZ, const double targetDistance)
{
    double tarz = fabs(_localCentroid.z),
        tary = fabs(_localCentroid.y);
    
    geometry_msgs::Vector3 msg;
    msg.x = 0;
    msg.y = tary;
    msg.z = tarz;
    _pubTargetError.publish(msg);

    return tarz < minZ 
        && tary < minY;
}

bool isRelativeDistanceValid(const double checkDistance)
{
    return !isnan(checkDistance) && checkDistance != INVALID_DISTANCE;
}

bool subscribedTopicsActive() 
{
    double currentTime = ros::Time::now().toSec();
    double dt_odom = currentTime - _timeLastOdometry;
    double dt_contour = currentTime - _timeLastContour;
    double dt_centLocal = currentTime - _timeLastCentroidLocal;
    //double dt_yaw = currentTime - _timeLastYawError;
    
    static constexpr double MAX_DT = 0.5;
    static constexpr double MAX_DT_SERVO = 5;
    ROS_FATAL_COND(dt_odom > MAX_DT,        "VSSM - odometry timeout reached.");
    ROS_FATAL_COND(dt_contour > MAX_DT,     "VSSM - contour timeout reached.");
    ROS_FATAL_COND(dt_centLocal > MAX_DT_SERVO,   "VSSM - centroid local timeout reached.");
    
    return dt_odom < MAX_DT 
        && dt_contour < MAX_DT 
        && dt_centLocal < MAX_DT_SERVO;
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
        

    double forward_movement_local = _descentSpeed * dt;
    double backward_movement_local = - _ascentSpeed * dt;

    switch (_currentState)
    {
        case BaloonPopState::OFF :
            // pass
            break;
        
        case BaloonPopState::ALIGNEMNT :
            //_currVisualServoFeed.x = _currDistanceReference;
            _currVisualServoFeed.x = _popXReference;
            _currVisualServoFeed.y = _popYReference;
            _currVisualServoFeed.yaw = 0;
            break;

        case BaloonPopState::POP :
            _popXReference += cos(-_uavYaw) * forward_movement_local;
            _popYReference += - sin(-_uavYaw) * forward_movement_local;
            
            _currVisualServoFeed.x = _popXReference;
            _currVisualServoFeed.y = _popYReference;
            _currVisualServoFeed.yaw = 0;
            _currDistanceReference = _currVisualServoFeed.x;
            break;

        case BaloonPopState::BACK :
            _popXReference += cos(-_uavYaw) * backward_movement_local;
            _popYReference += - sin(-_uavYaw) * backward_movement_local;
            _backStateTime += dt;

            _currVisualServoFeed.x = _popXReference;
            _currVisualServoFeed.y = _popYReference;
            _currVisualServoFeed.z = 0;
            _currVisualServoFeed.yaw = 0;
            break;
    }

    _currVisualServoFeed.header.stamp = ros::Time::now();
    _pubVisualServoFeed.publish(_currVisualServoFeed);

    // Publish currrent state
    std_msgs::Int32 stateMsg;
    stateMsg.data = static_cast<int>(_currentState);
    _pubVssmState.publish(stateMsg);
}

void odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    _currOdom = *msg;
    _timeLastOdometry = ros::Time::now().toSec();
    double _qx = _currOdom.pose.pose.orientation.x;
    double _qy = _currOdom.pose.pose.orientation.y;
    double _qz = _currOdom.pose.pose.orientation.z;
    double _qw = _currOdom.pose.pose.orientation.w;
    _uavYaw = atan2( 2 * (_qw * _qz + _qx * _qy), _qw * _qw + _qx * _qx - _qy * _qy - _qz * _qz);
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

        loopRate.sleep();
    }
}

private:

    static constexpr double THROTTLE_TIME = 3.0;
    double _rate = 50;

    /* Service baloon pop */
	ros::ServiceServer _serviceBrickPickup;
    bool _baloonPopActivated = false;
    BaloonPopState _currentState = BaloonPopState::OFF;
    
    /* Client for calling visual servo */
    ros::ServiceClient _vsClienCaller;

    /* Offset subscriber and publisher */
    ros::Publisher _pubVssmState;

    /* Pose publisher */
    ros::Publisher _pubVisualServoFeed;
    ros::Publisher _pubVelError, _pubTargetError;
    ros::Publisher _pubVisualServoSuccess;
    uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;
    nav_msgs::Odometry _currOdom;

    ros::Subscriber _subTrajectory;
    trajectory_msgs::MultiDOFJointTrajectoryPoint _trajPoint;

    /* Yaw error subscriber */
    //ros::Subscriber _subYawError;
    double _currUavVelError = 1e5;
    double _minTargetError,   
        _minTouchdownTargetPositionError_xy, _minTouchdownUavVelocityError_xy,
        _minTouchdownTargetPositionError_z, _minTouchdownUavVelocityError_z,
        _minTouchdownAlignDuration, _brickAlignHeight;
    ros::Subscriber _subPatchCentroid_local;
    geometry_msgs::Vector3 _globalCentroid, _localCentroid;    
    double _relativeBaloonDistance = INVALID_DISTANCE;
    double _popXReference, _popYReference;

    /* Contour subscriber */
    ros::Subscriber _subNContours;
    int _nContours;

    /* Touchdown mode parameters */
    double _touchdownHeight, _touchdownDelta = 0, _magnetOffset, 
        _backStateDuration = 0, _touchdownAlignDuration = 0, 
        _backStateTime, _touchdownSpeed, _visualServoDisableHeight;
    double _currDistanceReference, _descentSpeed, _ascentSpeed, _afterTouchdownHeight, _afterTouchdownHeight_GPS;
    double _uavYaw;
    int _descentTransitionCounter = 0, _descentCounterMax;

    double _timeLastContour = 0,
        _timeLastOdometry = 0,
        _timeLastCentroidLocal = 0;
        //_timeLastYawError = 0;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _vssmConfigMutex;
    dynamic_reconfigure::Server<vssm_param_t>
        _vssmConfigServer {_vssmConfigMutex, ros::NodeHandle(VSSM_DYN_RECONF)};
    dynamic_reconfigure::Server<vssm_param_t>::CallbackType _vssmParamCallback;
};
}

#endif /* VISUAL_SERVO_STATE_MACHINE_H */