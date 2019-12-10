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
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_control/VisualServoPursuitParametersConfig.h>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/filters/NonlinearFilters.h>

namespace uav_reference 
{

typedef uav_ros_control::VisualServoPursuitParametersConfig pursuit_param_t;
#define PURSUIT_DYN_RECONF              "pursuit_state_machine"
#define PARAM_RATE                      "pursuit/state_machine/rate"
#define INVALID_DISTANCE -1

enum PursuitState {
    OFF,
    UAV_FOLLOWING
};

class PursuitStateMachine
{

public:

PursuitStateMachine(ros::NodeHandle& nh)
{   
	initializeParameters(nh);
    // Define Publishers
    _pubVisualServoFeed = nh.advertise<uav_ros_control_msgs::VisualServoProcessValues>("visual_servo/process_value", 1);
    _pubOffsetX = nh.advertise<std_msgs::Float32>("visual_servo/offset_x", 1);
    _pubVssmState = nh.advertise<std_msgs::Int32>("visual_servo_sm/state", 1);

    // Define Subscribers
    _subOdom =
        nh.subscribe("odometry", 1, &uav_reference::PursuitStateMachine::odomCb, this);
    _subUAVDist = 
        nh.subscribe("uav/distance", 1, &uav_reference::PursuitStateMachine::brickDistCb, this);
  
    // Setup dynamic reconfigure server
	pursuit_param_t  pursuitConfig;
	setPursuitParameters(pursuitConfig);
	_pursuitConfigServer.updateConfig(pursuitConfig);
	_pursuitParamCallback = boost::bind(
		&uav_reference::PursuitStateMachine::pursuitParamCb, this, _1, _2);
	_pursuitConfigServer.setCallback(_pursuitParamCallback);

    // Setup brick pickup service callback
    _servicePursuit = nh.advertiseService(
			"pursuit",
			&uav_reference::PursuitStateMachine::pursuitServiceCb,
			this);

    // Initialize visual servo client caller
    _vsClienCaller = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("visual_servo");
}

~PursuitStateMachine()
{}

void brickDistCb(const std_msgs::Float32ConstPtr& msg)
{
    _relativeUAVDistance = msg->data;
}

bool pursuitServiceCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    if (false) // TODO: Handle case when pursuit should not turn on
    {
        turnOffVisualServo();
        _pursuitActivated = false;
        response.success = false;
        response.message = "Visual servo and brick pickup deactivated";
        return true;
    }

    // Check if brick pickup is already activated.
    if (_pursuitActivated)
    {
        ROS_FATAL("PSM::pursuitServiceCb - Puruist is already active.");
        response.success = true;
        response.message = "Pursuit is already active";
        return true;
    }

    // Try calling visual servo
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = true;
    if (!_vsClienCaller.call(req, resp))
    {
        ROS_FATAL("PSM::pursuitServiceCb - calling visual servo failed.");
        response.success = false;
        response.message = "Service caller for visual servo failed.";
        _currentState = PursuitState::OFF;
        return true;
    }

    if (resp.success)
    {
        // Visual servo successfully activated
        ROS_INFO("PSM::pursuitServiceCb() - pursuit is activated.");
        response.success = true;
        response.message = "Visual servo enabled - pursuit activated.";
        _pursuitActivated = true;
        return true;
    }
    
    ROS_WARN("PSM::pursuitServiceCb - unable to activate brick pickup.");
    response.success = false;
    response.message = "Visual servo failed to start - brick pickup inactive.";
    _pursuitActivated = false;

    return true;
}

void pursuitParamCb(pursuit_param_t& configMsg,uint32_t level)
{
    ROS_WARN("PursuitStateMachine::pursuitParamCb()");
    // TODO: Change internal parameter here from dyn_reconf configMsg
}

void setPursuitParameters(pursuit_param_t& config)
{   
    // TODO: Set initial reconfigure paramters here
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("PursuitStateMachine::initializeParameters()");
    bool initialized = nh.getParam(PARAM_RATE, _rate);
    // TODO: Load all the yaml parameters here 
    // Tip: Define parameter name at the top of the file

    ROS_INFO("Node rate: %.2f", _rate);
    if (!initialized)
	{
		ROS_FATAL("PursuitStateMachine::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("PursuitStateMachine parameters not properly initialized.");
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
        _currentState = PursuitState::OFF;
        _pursuitActivated = false; 
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
    // TODO: DO state machine logic here

    /*
    // If visual servo is inactive, deactivate state machine
    if (_currentState != PursuitState::OFF && !_pursuitActivated)
    {
        ROS_WARN("VSSM::updateStatus - Visual servo is inactive.");
        _currentState = PursuitState::OFF;
        _pursuitActivated = false;
        turnOffVisualServo();
        ROS_WARN("VSSM::updateStatus - OFF State activated.");
        return;
    }

    // If brick pickup is activate start brick alignment first
    if (_currentState == PursuitState::OFF && _pursuitActivated)
    {
        ROS_INFO("VSSM::updateStatus - Brick pickup requested");
        _currHeightReference = _currOdom.pose.pose.position.z;
        _descentTransitionCounter = 0;
        _currentState = PursuitState::BRICK_ALIGNMENT;
        ROS_INFO("VSSM::updateStatus - BRICK_ALIGNMENT state activated with height: %2f.", _currHeightReference);
        return;
    }

    // Update the transition counter
    if (_currentState == PursuitState::BRICK_ALIGNMENT &&
        sqrt(pow(_currTargetErrorX, 2) + pow(_currTargetErrorY, 2)) < _minTargetError && 
        abs(_currYawError) < _minYawError) _descentTransitionCounter++;

    // If brick alignemnt is activated and target error is withing range start descent
    if (_currentState == PursuitState::BRICK_ALIGNMENT &&
        _descentTransitionCounter > 100)
    {
        _currentState = PursuitState::DESCENT;
        ROS_INFO("VSSM::updateStatus - DESCENT state activated");
        return;
    }

    // if height is below touchdown treshold start touchdown
    if (_currentState == PursuitState::DESCENT &&
        isRelativeDistanceValid() && 
        _relativeUAVDistance <= _touchdownHeight)
    {
        _currentState = PursuitState::TOUCHDOWN;
        _touchdownTime = 0;
        _currHeightReference = _relativeUAVDistance;
        ROS_INFO("VSSM::UpdateStatus - TOUCHDOWN state activated");
        return;
    }

    // If touchdown time is exceeded, touchdown state is considered finished
    if (_currentState == PursuitState::TOUCHDOWN &&
        _touchdownTime >= _touchdownDuration && 
        _currHeightReference >= _afterTouchdownHeight) 
    {
        ROS_INFO("VSSM::updateStatus - Touchdown duration finished.");
        turnOffVisualServo();
    }
    */
}   

bool isRelativeDistanceValid()
{
    return _relativeUAVDistance > 0;
}

void publishVisualServoSetpoint(double dt)
{
    switch (_currentState)
    {
        case PursuitState::OFF :
            _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
            _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
            _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
            _currVisualServoFeed.yaw = util::calculateYaw(
                _currOdom.pose.pose.orientation.x,
                _currOdom.pose.pose.orientation.y,
                _currOdom.pose.pose.orientation.z,
                _currOdom.pose.pose.orientation.w);
            break;
        
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
    std_msgs::Float32 offsetXMsg;
    offsetXMsg.data = -1; // TODO: Put some value here 
    _pubOffsetX.publish(offsetXMsg);
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
        updateState();
        publishOffsets();
        publishVisualServoSetpoint(dt);
        loopRate.sleep();
    }
}

private:

    double _rate = 50;

    /* Service brick pickup */
	ros::ServiceServer _servicePursuit;
    bool _pursuitActivated = false;
    PursuitState _currentState = PursuitState::OFF;
    
    /* Client for calling visual servo */
    ros::ServiceClient _vsClienCaller;

    /* Offset subscriber and publisher */
    ros::Publisher _pubVssmState, _pubOffsetX;

    /* Pose publisher */
    ros::Publisher _pubVisualServoFeed;
    uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;
    nav_msgs::Odometry _currOdom;

    ros::Subscriber _subUAVDist;
    double _relativeUAVDistance = INVALID_DISTANCE;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _pursuitConfigMutex;
    dynamic_reconfigure::Server<pursuit_param_t>
        _pursuitConfigServer {_pursuitConfigMutex, ros::NodeHandle(PURSUIT_DYN_RECONF)};
    dynamic_reconfigure::Server<pursuit_param_t>::CallbackType _pursuitParamCallback;
};
}

#endif /* PURSUIT_STATE_MACHINE */