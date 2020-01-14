#include <uav_ros_control/control/CascadePID.h>
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Define all parameter paths here
#define PID_X_PARAM "control/pos_x"
#define PID_Y_PARAM "control/pos_y"
#define PID_Z_PARAM "control/pos_z"
#define PID_VX_PARAM "control/vel_x"
#define PID_VY_PARAM "control/vel_y"
#define PID_VZ_PARAM "control/vel_z"
#define FFGAIN_VEL_PARAM "control/ff_gain/velocity"
#define FFGAIN_ACC_PARAM "control/ff_gain/acceleration"
#define HOVER_PARAM "control/hover"
#define GRAVITY_ACCELERATION 9.8;

uav_controller::CascadePID::CascadePID(ros::NodeHandle& nh) :
    _posXPID {new PID ("Position - x")},
    _posYPID {new PID ("Position - y")}, 
    _posZPID {new PID ("Position - z")},
    _velXPID {new PID ("Velocity - x")},
    _velYPID {new PID ("Velocity - y")},
    _velZPID {new PID ("Velocity - z")}, 
    uav_controller::ControlBase(nh)
{
	// Initialize class parameters
	initializeParameters(nh);
	_velRefPub = nh.advertise<geometry_msgs::Vector3>("carrot/velocity", 1);
	_velCurrPub = nh.advertise<geometry_msgs::Vector3>("uav/velocity", 1);
	_yawRefSub = nh.subscribe("carrot/yaw", 1, &uav_controller::CascadePID::yawRefCb, this);

	// Setup dynamic reconfigure server
	uav_ros_control::PositionControlParametersConfig posConfig;
	setPositionReconfigureParams(posConfig);
	_posConfigServer.updateConfig(posConfig);
	_posParamCallback = boost::bind(
		&uav_controller::CascadePID::positionParamsCb, this, _1, _2);
	_posConfigServer.setCallback(_posParamCallback);

	// Initialize position hold service
	_serviceResetIntegrators = nh.advertiseService(
			"reset_integrator",
			&uav_controller::CascadePID::intResetServiceCb,
			this);
}

bool uav_controller::CascadePID::intResetServiceCb(std_srvs::Empty::Request& request, 
	std_srvs::Empty::Response& response)
{
	ROS_WARN("Resetting all PID controllers");
	resetPositionPID();
	resetVelocityPID();
	return true;
}
	
uav_controller::CascadePID::~CascadePID()
{
}

void uav_controller::CascadePID::yawRefCb(const std_msgs::Float64ConstPtr& msg)
{
	_yawRef = msg->data;
}

void uav_controller::CascadePID::positionParamsCb(
		uav_ros_control::PositionControlParametersConfig& configMsg,
		uint32_t level)
{
	ROS_WARN("CascadePID::parametersCallback");

	_posYPID->set_kp(configMsg.k_p_xy);
	_posYPID->set_kd(configMsg.k_d_xy);
	_posYPID->set_ki(configMsg.k_i_xy);
	_posYPID->set_lim_high(configMsg.lim_high_xy);
	_posYPID->set_lim_low(configMsg.lim_low_xy);

	_velYPID->set_kp(configMsg.k_p_vxy);
	_velYPID->set_kd(configMsg.k_d_vxy);
	_velYPID->set_ki(configMsg.k_i_vxy);
	_velYPID->set_lim_high(configMsg.lim_high_vxy);
	_velYPID->set_lim_low(configMsg.lim_low_vxy);
	
	_posXPID->set_kp(configMsg.k_p_xy);
	_posXPID->set_kd(configMsg.k_d_xy);
	_posXPID->set_ki(configMsg.k_i_xy);
	_posXPID->set_lim_high(configMsg.lim_high_xy);
	_posXPID->set_lim_low(configMsg.lim_low_xy);

	_velXPID->set_kp(configMsg.k_p_vxy);
	_velXPID->set_kd(configMsg.k_d_vxy);
	_velXPID->set_ki(configMsg.k_i_vxy);
	_velXPID->set_lim_high(configMsg.lim_high_vxy);
	_velXPID->set_lim_low(configMsg.lim_low_vxy);

	_posZPID->set_kp(configMsg.k_p_z);
	_posZPID->set_kd(configMsg.k_d_z);
	_posZPID->set_ki(configMsg.k_i_z);
	_posZPID->set_lim_high(configMsg.lim_high_z);
	_posZPID->set_lim_low(configMsg.lim_low_z);

	_velZPID->set_kp(configMsg.k_p_vz);
	_velZPID->set_kd(configMsg.k_d_vz);
	_velZPID->set_ki(configMsg.k_i_vz);
	_velZPID->set_lim_high(configMsg.lim_high_vz);
	_velZPID->set_lim_low(configMsg.lim_low_vz);

	_hoverThrust = configMsg.hover;
}

void uav_controller::CascadePID::setPositionReconfigureParams(
	uav_ros_control::PositionControlParametersConfig& config)
{
    ROS_WARN("CascadePID::setReconfigureParameters");

	config.k_p_xy = _posYPID->get_kp();
	config.k_i_xy = _posYPID->get_ki();
	config.k_d_xy = _posYPID->get_kd();
	config.lim_low_xy = _posYPID->get_lim_low();
	config.lim_high_xy = _posYPID->get_lim_high();
	
	config.k_p_vxy = _velYPID->get_kp();
	config.k_i_vxy = _velYPID->get_ki();
	config.k_d_vxy = _velYPID->get_kd();
	config.lim_low_vxy = _velYPID->get_lim_low();
	config.lim_high_vxy = _velYPID->get_lim_high();
	
	config.k_p_z = _posZPID->get_kp();
	config.k_i_z = _posZPID->get_ki();
	config.k_d_z = _posZPID->get_kd();
	config.lim_low_z = _posZPID->get_lim_low();
	config.lim_high_z = _posZPID->get_lim_high();

	config.k_p_vz = _velZPID->get_kp();
	config.k_i_vz = _velZPID->get_ki();
	config.k_d_vz = _velZPID->get_kd();
	config.lim_low_vz = _velZPID->get_lim_low();
	config.lim_high_vz = _velZPID->get_lim_high();

	config.hover = _hoverThrust;
}

void uav_controller::CascadePID::initializeParameters(ros::NodeHandle& nh)
{
	ROS_WARN("CascadePID::initializeParameters()");

    _posYPID->initializeParameters(nh, PID_Y_PARAM);
	_velYPID->initializeParameters(nh, PID_VY_PARAM);
	_posXPID->initializeParameters(nh, PID_X_PARAM);
	_velXPID->initializeParameters(nh, PID_VX_PARAM);
	_posZPID->initializeParameters(nh, PID_Z_PARAM);
	_velZPID->initializeParameters(nh, PID_VZ_PARAM);

	bool initialized = nh.getParam(HOVER_PARAM, _hoverThrust)
		&& nh.getParam(FFGAIN_VEL_PARAM, _ffGainVelocity)
		&& nh.getParam(FFGAIN_ACC_PARAM, _ffGainAcceleration);
	ROS_INFO("Hover thrust: %.2f", _hoverThrust);
	ROS_INFO("Feed-forward velocity gain: %.2f", _ffGainVelocity);
	ROS_INFO("Feed-forward acceleration gain: %.2f", _ffGainAcceleration);
	if (!initialized)
	{
		ROS_FATAL("CascadePID::initalizeParameters() - failed to initialize parameters");
		throw std::runtime_error("CascadedPID parameters not properly initialized.");
	}
}

void uav_controller::CascadePID::resetPositionPID()
{
	_posXPID->resetIntegrator();
	_posYPID->resetIntegrator();
	_posZPID->resetIntegrator();
}

void uav_controller::CascadePID::resetVelocityPID()
{
	_velXPID->resetIntegrator();
	_velYPID->resetIntegrator();
	_velZPID->resetIntegrator();
}

void uav_controller::CascadePID::calculateAttThrustSp(double dt)
{
	// Calculate first row of PID controllers
	double velocityRefX = _posXPID->compute(
		getCurrentReference().transforms[0].translation.x, getCurrPosition()[0], dt);
	double velocityRefY = _posYPID->compute(
		getCurrentReference().transforms[0].translation.y, getCurrPosition()[1], dt);
	double velocityRefZ = _posZPID->compute(
		getCurrentReference().transforms[0].translation.z, getCurrPosition()[2], dt);

	// Add velocity feed-forward gains
	velocityRefX += _ffGainVelocity * getCurrentReference().velocities[0].linear.x;
	velocityRefY += _ffGainVelocity * getCurrentReference().velocities[0].linear.y;
	velocityRefZ += _ffGainVelocity * getCurrentReference().velocities[0].linear.z;

	// Calculate second row of PID controllers
	double roll = - _velYPID->compute(velocityRefY, getCurrVelocity()[1], dt);
	double pitch = _velXPID->compute(velocityRefX, getCurrVelocity()[0], dt);
	double thrust = _velZPID->compute(velocityRefZ, getCurrVelocity()[2], dt);
	thrust += _hoverThrust;

	// Add acceleration feed-forward gains
	roll += _ffGainAcceleration * 
		getCurrentReference().accelerations[0].linear.x / GRAVITY_ACCELERATION;
	pitch += _ffGainAcceleration * 
		getCurrentReference().accelerations[0].linear.y / GRAVITY_ACCELERATION;
	_yawRef += _ffGainAcceleration *
		getCurrentReference().accelerations[0].linear.z / GRAVITY_ACCELERATION;

	// Decouple roll and pitch w.r.t. yaw
	setAttitudeSp( 
		cos(getCurrentYaw()) * roll + sin(getCurrentYaw()) * pitch, 
		cos(getCurrentYaw()) * pitch - sin(getCurrentYaw()) * roll, 
		_yawRef);
	
	setThrustSp(thrust);

	geometry_msgs::Vector3 newMsg;
	newMsg.x = velocityRefX;
	newMsg.y = velocityRefY;
	newMsg.z = velocityRefZ;
	_velRefPub.publish(newMsg);

	geometry_msgs::Vector3 vel;
	vel.x = getCurrVelocity()[0];
	vel.y = getCurrVelocity()[1];
	vel.z = getCurrVelocity()[2];
	_velCurrPub.publish(vel);
}

void uav_controller::runDefault(
	uav_controller::CascadePID& cascadeObj, ros::NodeHandle& nh)
{
	double rate = 50;
	double dt = 1.0/rate;
	ros::Rate loopRate(rate);
	
	while (ros::ok())
	{
		ros::spinOnce();
		cascadeObj.calculateAttThrustSp(dt);
		cascadeObj.publishAttitudeTarget(MASK_IGNORE_RPY_RATE);
		cascadeObj.publishEulerSp();
		loopRate.sleep();
	}
}