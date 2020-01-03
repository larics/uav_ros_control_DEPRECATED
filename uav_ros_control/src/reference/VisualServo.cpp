//
// Created by robert on 20. 09. 2019..
//

#include <uav_ros_control/reference/VisualServo.h>
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <math.h>
#include <math.h>

// Define all parameter paths here
#define VS_P_GAIN_X_PARAM          "reference/p_gain_x"
#define VS_I_GAIN_X_PARAM          "reference/i_gain_x"
#define VS_D_GAIN_X_PARAM          "reference/d_gain_x"
#define VS_I_CLAMP_X_PARAM         "reference/i_clamp_x"
#define VS_OFFSET_X_1_PARAM        "reference/offset_x_1"
#define VS_OFFSET_X_2_PARAM        "reference/offset_x_2"
#define VS_DEADZONE_X_PARAM        "reference/deadzone_x"
#define VS_LANDING_RANGE_X_PARAM   "reference/landing_range_x"

#define VS_P_GAIN_Y_PARAM          "reference/p_gain_y"
#define VS_I_GAIN_Y_PARAM          "reference/p_gain_y"
#define VS_D_GAIN_Y_PARAM          "reference/d_gain_y"
#define VS_I_CLAMP_Y_PARAM         "reference/i_clamp_y"
#define VS_OFFSET_Y_1_PARAM        "reference/offset_y_1"
#define VS_OFFSET_Y_2_PARAM        "reference/offset_y_2"
#define VS_DEADZONE_Y_PARAM        "reference/deadzone_y"
#define VS_LANDING_RANGE_Y_PARAM   "reference/landing_range_y"

#define VS_P_GAIN_Z_PARAM          "reference/p_gain_z"
#define VS_I_GAIN_Z_PARAM          "reference/p_gain_z"
#define VS_D_GAIN_Z_PARAM          "reference/d_gain_z"
#define VS_I_CLAMP_Z_PARAM         "reference/i_clamp_z"
#define VS_OFFSET_Z_1_PARAM        "reference/offset_z_1"
#define VS_OFFSET_Z_2_PARAM        "reference/offset_z_2"
#define VS_DEADZONE_Z_PARAM        "reference/deadzone_z"
#define VS_LANDING_RANGE_Z_PARAM   "reference/landing_range_z"

#define VS_P_GAIN_YAW_PARAM        "reference/p_gain_yaw"
#define VS_I_GAIN_YAW_PARAM        "reference/i_gain_yaw"
#define VS_I_CLAMP_YAW_PARAM       "reference/i_clamp_yaw"
#define VS_I_DEADZONE_YAW_PARAM    "reference/i_deadzone_yaw"
#define VS_D_GAIN_YAW_PARAM        "reference/d_gain_yaw"
#define VS_LANDING_RANGE_YAW_PARAM "reference/lending_range_yaw"

#define VS_P_GAIN_DIST_PARAM       "reference/p_gain_dist"
#define VS_I_GAIN_DIST_PARAM       "reference/i_gain_dist"
#define VS_D_GAIN_DIST_PARAM       "reference/d_gain_dist"
#define VS_I_CLAMP_DIST_PARAM      "reference/i_clamp_dist"
#define VS_DEADZONE_DIST_PARAM     "reference/deadzone_dist"

#define VS_MOVE_SATURATION_PARAM   "reference/move_saturation"
#define VS_YAW_DIFFERENCE_PARAM    "reference/yaw_difference"

#define VS_LANDING_SPEED_PARAM     "reference/landing_speed"
#define VS_IS_BRICK_LAYING_PARAM   "reference/is_brick_laying"

namespace uav_reference {

VisualServo::VisualServo(ros::NodeHandle& nh) {
  initializeParameters(nh);
  ros::Duration(2.0).sleep();

  // Define Publishers
  _pubXError = nh.advertise<std_msgs::Float32>("visual_servo/x_error", 1);
  _pubYError = nh.advertise<std_msgs::Float32>("visual_servo/y_error", 1);
  _pubZError = nh.advertise<std_msgs::Float32>("visual_servo/z_error", 1);
  _pubIsEnabledTopic = nh.advertise<std_msgs::Bool>("visual_servo/is_enabled", 1);
  _pubMoveLeft = nh.advertise<std_msgs::Float32>("move_left", 1);
  _pubChangeYaw = nh.advertise<std_msgs::Float32>("change_yaw", 1);
  _pubMoveForward = nh.advertise<std_msgs::Float32>("move_forward", 1);
  _pubMoveUp = nh.advertise<std_msgs::Float32>("move_up", 1);
  
  _pubUavYawDebug = nh.advertise<std_msgs::Float32>("debug/Uav_yaw", 1);
  _pubYawErrorDebug = nh.advertise<std_msgs::Float32>("debug/yaw_error", 1);
  _pubChangeYawDebug = nh.advertise<std_msgs::Float32>("debug/yaw_change", 1); // Advertised again for user friendliness
  _pubUavRollDebug = nh.advertise<std_msgs::Float32>("debug/uav_roll", 1);
  _pubUavPitchDebug = nh.advertise<std_msgs::Float32>("debug/uav_pitch", 1);
  _pubNewSetpoint =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory", 1);

  // DEBUG
  _pubXAdd = nh.advertise<std_msgs::Float32>("debug/x_added", 1);

  _pubYAdd = nh.advertise<std_msgs::Float32>("debug/y_added", 1);

  // Define Subscribers
  _subOdom =
      nh.subscribe("odometry", 1, &uav_reference::VisualServo::odomCb, this);
  _subXError =
      nh.subscribe("x_error", 1, &uav_reference::VisualServo::xErrorCb, this);
  _subYError =
      nh.subscribe("y_error", 1, &uav_reference::VisualServo::yErrorCb, this);
  _subZError = 
      nh.subscribe("z_error", 1, &uav_reference::VisualServo::zErrorCb, this);
  _subYawError =
      nh.subscribe("yaw_error", 1, &uav_reference::VisualServo::yawErrorCb, this);
  _subXOffset =
      nh.subscribe("x_offset", 1, &uav_reference::VisualServo::xOffsetCb, this);
  _subYOffset =
      nh.subscribe("y_offset", 1, &uav_reference::VisualServo::yOffsetCb, this);
  _subZOffset =
      nh.subscribe("z_offset", 1, &uav_reference::VisualServo::zOffsetCb, this);
            
  _subVisualServoProcessValuesMsg =
      nh.subscribe("VisualServoProcessValueTopic", 1, &uav_reference::VisualServo::VisualServoProcessValuesCb, this);

  // Setup dynamic reconfigure
  _VSParamCallback = boost::bind(&VisualServo::visualServoParamsCb, this, _1, _2);
  _VSConfigServer.setCallback(_VSParamCallback);

  _serviceStartVisualServo = nh.advertiseService(
      "visual_servo",
      &uav_reference::VisualServo::startVisualServoServiceCb,
      this);

  _new_point.transforms = std::vector<geometry_msgs::Transform>(1);
  _new_point.velocities = std::vector<geometry_msgs::Twist>(1);
  _new_point.accelerations = std::vector<geometry_msgs::Twist>(1);
}

VisualServo::~VisualServo() {}

void uav_reference::VisualServo::initializeParameters(ros::NodeHandle& nh)
{
  ROS_WARN("CascadePID::initializeParameters()");

  bool x_armed = false, y_armed = false, z_armed = false, yaw_armed = false;
  bool initialized = nh.getParam("visual_servo/camera_fov", _camera_fov) &&
    nh.getParam("visual_servo/compensate_roll_and_pitch", _compensate_roll_and_pitch) && 
    nh.getParam("visual_servo/pid_x/x_armed", x_armed) &&
    nh.getParam("visual_servo/pid_y/y_armed", y_armed) &&
    nh.getParam("visual_servo/pid_z/z_armed", z_armed) &&
    nh.getParam("visual_servo/pid_yaw/yaw_armed", yaw_armed) &&
    nh.getParam("visual_servo/pid_x/deadzone_x", _deadzone_x) &&
    nh.getParam("visual_servo/pid_y/deadzone_y", _deadzone_y) &&
    nh.getParam("visual_servo/pid_z/deadzone_z", _deadzone_z) &&
    nh.getParam("visual_servo/pid_yaw/deadzone_yaw", _deadzone_yaw) &&
    nh.getParam("visual_servo/yaw_added_offset", _yaw_added_offset);
  
  ROS_INFO_COND(_compensate_roll_and_pitch, "VS - Roll and pitch compensation is active");
  ROS_INFO("VS - camera FOV %.2f", _camera_fov);
  ROS_INFO("VS - deadzones x,y,z,yaw = [%.3f, %.3f, %.3f, %.3f]", 
    _deadzone_x, _deadzone_y, _deadzone_z, _deadzone_yaw);
  ROS_INFO("Yaw added offset: %.2f", _yaw_added_offset);
  
  if (x_armed)
    _x_axis_PID.initializeParameters(nh, "visual_servo/pid_x");
  if (y_armed)
    _y_axis_PID.initializeParameters(nh, "visual_servo/pid_y");
  if (z_armed)
    _z_axis_PID.initializeParameters(nh, "visual_servo/pid_z");
  if (yaw_armed)
    _yaw_PID.initializeParameters(nh, "visual_servo/pid_yaw");

    if (!initialized)
    {
      ROS_FATAL("VisualServo::initalizeParameters() - failed to initialize parameters");
		  throw std::runtime_error("VisualServo parameters not properly initialized.");
    }

  uav_ros_control::VisualServoParametersConfig cfg;
  _VSConfigServer.getConfigDefault(cfg);

  cfg.x_armed = x_armed;
  if (x_armed)
  {
    cfg.k_p_x = _x_axis_PID.get_kp();
    cfg.k_i_x = _x_axis_PID.get_ki();
    cfg.k_d_x = _x_axis_PID.get_kd();  
    cfg.saturation_x = _x_axis_PID.get_lim_high();
    cfg.deadzone_x = _deadzone_x;
  }

  cfg.y_armed = y_armed;  
  if (y_armed)
  {
    cfg.k_p_y = _y_axis_PID.get_kp();
    cfg.k_i_y = _y_axis_PID.get_ki();
    cfg.k_d_y = _y_axis_PID.get_kd();  
    cfg.saturation_y = _y_axis_PID.get_lim_high();
    cfg.deadzone_y = _deadzone_y;
  }

  cfg.z_armed = z_armed;  
  if (z_armed)
  {
    cfg.k_p_z = _z_axis_PID.get_kp();
    cfg.k_i_z = _z_axis_PID.get_ki();
    cfg.k_d_z = _z_axis_PID.get_kd();  
    cfg.saturation_z = _z_axis_PID.get_lim_high();
    cfg.deadzone_z = _deadzone_z;
  }

  cfg.yaw_armed = yaw_armed;  
  if (yaw_armed)
  {
    cfg.k_p_yaw = _yaw_PID.get_kp();
    cfg.k_i_yaw = _yaw_PID.get_ki();
    cfg.k_d_yaw = _yaw_PID.get_kd();  
    cfg.saturation_yaw = _yaw_PID.get_lim_high();
    cfg.deadzone_yaw = _deadzone_yaw;
  }

  cfg.camera_fov = _camera_fov;
  cfg.compensate_roll_and_pitch = _compensate_roll_and_pitch;
  cfg.yaw_added_offset = _yaw_added_offset;
  _VSConfigServer.updateConfig(cfg);
}


bool uav_reference::VisualServo::startVisualServoServiceCb(std_srvs::SetBool::Request &request,
                                                           std_srvs::SetBool::Response &response) {
  if (request.data) {
    if (!isVisualServoEnabled()) ROS_INFO("UAV VisualServo - enabling visual servo.");
    _visualServoEnabled = true;
    response.message = "Visual servo enabled.";
  }
  else {
    if(isVisualServoEnabled()) ROS_INFO("UAV VisualServo - disabling visual servo.");
    _visualServoEnabled = false;
    _yaw_PID.resetIntegrator();
    _x_axis_PID.resetIntegrator();
    _y_axis_PID.resetIntegrator();
    response.message = "Visual servo disabled.";
  }

  _x_frozen = false;
  _y_frozen = false;
  _z_frozen = false;
  _yaw_frozen = false;
  response.success = _visualServoEnabled;
  return true;
}

void VisualServo::visualServoParamsCb(uav_ros_control::VisualServoParametersConfig &configMsg,
                                                     uint32_t level) {
  ROS_WARN("VisualServo::parametersCallback");

  _deadzone_x = configMsg.deadzone_x;
  _deadzone_y = configMsg.deadzone_y;
  _deadzone_yaw  = configMsg.deadzone_yaw;
  _deadzone_z = configMsg.deadzone_z;
   
  _x_axis_PID.set_kp(configMsg.k_p_x);
  _x_axis_PID.set_ki(configMsg.k_i_x);
  _x_axis_PID.set_kd(configMsg.k_d_x);
  _x_axis_PID.set_lim_high(configMsg.saturation_x);
  _x_axis_PID.set_lim_low(-configMsg.saturation_x);

  if (!configMsg.x_armed) {
    _x_axis_PID.set_kp(0);
    _x_axis_PID.set_ki(0);
    _x_axis_PID.set_kd(0);
    _x_axis_PID.resetIntegrator();
  }

  _y_axis_PID.set_kp(configMsg.k_p_y);
  _y_axis_PID.set_ki(configMsg.k_i_y);
  _y_axis_PID.set_kd(configMsg.k_d_y);
  _y_axis_PID.set_lim_high(configMsg.saturation_y);
  _y_axis_PID.set_lim_low(-configMsg.saturation_y);

  if (!configMsg.y_armed) {
    _y_axis_PID.set_kp(0);
    _y_axis_PID.set_ki(0);
    _y_axis_PID.set_kd(0);
    _y_axis_PID.resetIntegrator();
  }

  _z_axis_PID.set_kp(configMsg.k_p_z);
  _z_axis_PID.set_ki(configMsg.k_i_z);
  _z_axis_PID.set_kd(configMsg.k_d_z);
  _z_axis_PID.set_lim_high(configMsg.saturation_z);
  _z_axis_PID.set_lim_low(-configMsg.saturation_z);

  if (!configMsg.z_armed) {
    _z_axis_PID.set_kp(0);
    _z_axis_PID.set_ki(0);
    _z_axis_PID.set_kd(0);
    _z_axis_PID.resetIntegrator();
  }

  _yaw_PID.set_kp(configMsg.k_p_yaw);
  _yaw_PID.set_ki(configMsg.k_i_yaw);
  _yaw_PID.set_kd(configMsg.k_d_yaw);
  _yaw_PID.set_lim_high(configMsg.saturation_yaw);
  _yaw_PID.set_lim_low(-configMsg.saturation_yaw);

  if (!configMsg.yaw_armed) {
    _yaw_PID.set_kp(0);
    _yaw_PID.set_ki(0);
    _yaw_PID.set_kd(0);
    _yaw_PID.resetIntegrator();
  }

  _compensate_roll_and_pitch = configMsg.compensate_roll_and_pitch;
  _camera_fov = configMsg.camera_fov * M_PI / 180.0;
  _yaw_added_offset = configMsg.yaw_added_offset;
}

void VisualServo::odomCb(const nav_msgs::OdometryConstPtr& odom) {

    _qx = odom->pose.pose.orientation.x;
    _qy = odom->pose.pose.orientation.y;
    _qz = odom->pose.pose.orientation.z;
    _qw = odom->pose.pose.orientation.w;

    _uavRoll = atan2( 2*(_qw * _qx + _qy * _qz), 1 - 2 * (_qx*_qx + _qy*_qy) );
    _uavPitch = asin( 2*(_qw*_qy - _qx*_qz) );

    _floatMsg.data = _uavRoll;
    _pubUavRollDebug.publish(_floatMsg);
    _floatMsg.data = _uavPitch;
    _pubUavPitchDebug.publish(_floatMsg);
}

void VisualServo::xErrorCb(const std_msgs::Float32 &data) {
  _error_x = data.data;

  if (_compensate_roll_and_pitch){
      _error_x += tan(_uavRoll)/tan(_camera_fov);
  }

  // _floatMsg.data = _error_x - _offset_x;
  _floatMsg.data = _offset_x - _error_x;
  _pubXError.publish(_floatMsg);
}

void VisualServo::yErrorCb(const std_msgs::Float32 &data) {
  _error_y = data.data;

  if(_compensate_roll_and_pitch) {
      _error_y += tan(_uavPitch)/tan(_camera_fov);
  }

  _floatMsg.data = _error_y - _offset_y;
  _pubYError.publish(_floatMsg);
}

void VisualServo::zErrorCb(const std_msgs::Float32& msg)
{
  _error_z = msg.data;

  _floatMsg.data = _error_z - _offset_z;
  _pubZError.publish(_floatMsg);
}

void VisualServo::yawErrorCb(const std_msgs::Float32 &data) {
  _error_yaw = util::wrapMinMax(-data.data - _yaw_added_offset, -M_PI_2, M_PI_2);
  std_msgs::Float32 m;
  m.data = _error_yaw;
  _pubYawErrorDebug.publish(m);
}

void VisualServo::VisualServoProcessValuesCb(const uav_ros_control_msgs::VisualServoProcessValues &msg) {
    if (msg.x == 0.0) {
        _x_frozen = true;
    }
    if (!_x_frozen) {
        _uavPos[0] = msg.x;
    }

    if (msg.y == 0.0) {
        _y_frozen = true;
    }
    if (!_y_frozen) {
        _uavPos[1] = msg.y;
    }

    if (msg.z == 0.0) {
      _z_frozen = true;
    }
    if (!_z_frozen) {
      _uavPos[2] = msg.z;
    }

    if (msg.yaw == 0.0) {
        _yaw_frozen = true;
    }
    if (!_yaw_frozen) {
        _uavYaw = msg.yaw;
    }
    _floatMsg.data = _uavYaw;
    _pubUavYawDebug.publish(_floatMsg);
}

void VisualServo::xOffsetCb(const std_msgs::Float32 &msg) {
    _offset_x = msg.data;
}

void VisualServo::yOffsetCb(const std_msgs::Float32 &msg){
    _offset_y = msg.data;
}

void VisualServo::zOffsetCb(const std_msgs::Float32 &msg){
    _offset_z = msg.data;
}

void VisualServo::updateSetpoint() {

  double move_forward = 0.0;
  double move_left = 0.0;
  double move_up = 0.0;
  double change_yaw = 0.0;

  if (!_x_frozen) move_left = _x_axis_PID.compute(_offset_x, _error_x, 1 / _rate);
  if (!_y_frozen) move_forward  = _y_axis_PID.compute(_offset_y, _error_y, 1 / _rate);
  if (!_z_frozen) move_up = _z_axis_PID.compute(_offset_z, _error_z, 1 / _rate);
  if (!_yaw_frozen) change_yaw = _yaw_PID.compute(0, _error_yaw, 1 / _rate);
  _floatMsg.data = change_yaw;
  _pubChangeYawDebug.publish(_floatMsg);

  _setpointPosition[0] = _uavPos[0] + move_forward * cos(_uavYaw + _yaw_added_offset);
  _setpointPosition[0] -= move_left * sin(_uavYaw + _yaw_added_offset);
  _setpointPosition[1] = _uavPos[1] + move_forward * sin(_uavYaw + _yaw_added_offset);
  _setpointPosition[1] += move_left * cos(_uavYaw + _yaw_added_offset);
  _setpointPosition[2] = _uavPos[2] + move_up;

  _setpointYaw = _uavYaw + change_yaw;

  _moveLeftMsg.data = move_left;
  _changeYawMsg.data = change_yaw;
  _moveUpMsg.data = move_up;
  _moveForwardMsg.data = move_forward;

  _pubMoveLeft.publish(_moveLeftMsg);
  _pubChangeYaw.publish(_changeYawMsg);
  _pubMoveForward.publish(_moveForwardMsg);
  _pubMoveUp.publish(_moveUpMsg);
  _moveLeftMsg.data = _setpointPosition[0]- _uavPos[0];
  _pubXAdd.publish(_moveLeftMsg);
  _moveLeftMsg.data = _setpointPosition[1]- _uavPos[1];
  _pubYAdd.publish(_moveLeftMsg);
}

void VisualServo::publishNewSetpoint() {

  tf2::Quaternion q;
  q.setEulerZYX(_setpointYaw, 0.0, 0.0);

  _new_point.transforms[0].translation.x = _setpointPosition[0];
  _new_point.transforms[0].translation.y = _setpointPosition[1];
  _new_point.transforms[0].translation.z = _setpointPosition[2];
  _new_point.transforms[0].rotation.x = q.getX();
  _new_point.transforms[0].rotation.y = q.getY();
  _new_point.transforms[0].rotation.z = q.getZ();
  _new_point.transforms[0].rotation.w = q.getW();

  _pubNewSetpoint.publish(_new_point);
}

void VisualServo::publishStatus() {
    _boolMsg.data = isVisualServoEnabled();
    _pubIsEnabledTopic.publish(_boolMsg);
}

bool VisualServo::isVisualServoEnabled() {
  return _visualServoEnabled;
}

void runDefault(VisualServo& visualServoRefObj, ros::NodeHandle& nh) {
  double rate = 50;
  visualServoRefObj.setRate(rate);
  ros::Rate loopRate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    visualServoRefObj.updateSetpoint();
    if (visualServoRefObj.isVisualServoEnabled()) {
      visualServoRefObj.publishNewSetpoint();
    }
    visualServoRefObj.publishStatus();
    loopRate.sleep();
  }
}
} // namespace uav_reference