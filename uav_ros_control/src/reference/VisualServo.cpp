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
  _pubMoveLeft = nh.advertise<std_msgs::Float32>("visual_servo/move_left", 1);
  _pubChangeYaw = nh.advertise<std_msgs::Float32>("visual_servo/change_yaw", 1);
  _pubMoveForward = nh.advertise<std_msgs::Float32>("visual_servo/move_forward", 1);
  _pubMoveUp = nh.advertise<std_msgs::Float32>("visual_servo/move_up", 1);
  
  _pubYawErrorDebug = nh.advertise<std_msgs::Float32>("debug/yaw_error", 1);
  _pubNewSetpoint =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory", 1);

  // DEBUG
  _pubXAdd = nh.advertise<std_msgs::Float32>("debug/x_added", 1);

  _pubYAdd = nh.advertise<std_msgs::Float32>("debug/y_added", 1);

  // RATE LIMITER
  _pubMoveForwardLimited = nh.advertise<std_msgs::Float32>("visual_servo/move_forward_limited", 1);
  _pubMoveUpLimited = nh.advertise<std_msgs::Float32>("visual_servo/move_up_limited", 1);
  _pubChangeYawLimited = nh.advertise<std_msgs::Float32>("visual_servo/change_yaw_limited", 1);

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

  _subCarrotTrajectory = nh.subscribe("carrot/trajectory", 1, &uav_reference::VisualServo::CarrotReferenceCb, this);

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

  //
  // MoveForwardRateLimiter.init(_DistanceRateLimiter_T, _DistanceRateLimiter_R, -_DistanceRateLimiter_R, 0.0);
  // ROS_FATAL("Initializing rate limiter");

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
    nh.getParam("visual_servo/yaw_added_offset", _yaw_added_offset) &&
    nh.getParam("visual_servo/filter_k", k) &&
    nh.getParam("visual_servo/rate_limiters/distance/rate_limiter_R", _DistanceRateLimiter_R) &&
    nh.getParam("visual_servo/rate_limiters/distance/rate_limiter_T", _DistanceRateLimiter_T) &&
    nh.getParam("visual_servo/rate_limiters/distance/rate_limiter_F", _DistanceRateLimiter_F) &&
    nh.getParam("visual_servo/rate_limiters/height/rate_limiter_R", _HeightRateLimiter_R) &&
    nh.getParam("visual_servo/rate_limiters/height/rate_limiter_T", _HeightRateLimiter_T) &&
    nh.getParam("visual_servo/rate_limiters/yaw/rate_limiter_R", _YawRateLimiter_R) &&
    nh.getParam("visual_servo/rate_limiters/yaw/rate_limiter_T", _YawRateLimiter_T) &&
    nh.getParam("visual_servo/rate_limiters/main/rate_limiter_R", _SetpointRateLimiter_R) &&
    nh.getParam("visual_servo/rate_limiters/main/rate_limiter_T", _SetpointRateLimiter_T);
  
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

  cfg.distance_R = _DistanceRateLimiter_R;
  cfg.distance_T = _DistanceRateLimiter_T;
  cfg.distance_F = _DistanceRateLimiter_F;
  cfg.height_R = _HeightRateLimiter_R;
  cfg.height_T = _HeightRateLimiter_T;
  cfg.yaw_R = _YawRateLimiter_R;
  cfg.yaw_T = _YawRateLimiter_T;

  cfg.main_R = _SetpointRateLimiter_R;
  cfg.main_T = _SetpointRateLimiter_T;

  cfg.camera_fov = _camera_fov;
  cfg.compensate_roll_and_pitch = _compensate_roll_and_pitch;
  cfg.yaw_added_offset = _yaw_added_offset;
  cfg.filter_k = k;
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

  MoveForwardRateLimiter.init(_DistanceRateLimiter_T, _DistanceRateLimiter_R, _DistanceRateLimiter_F, 0.0);
  MoveUpRateLimiter.init(_DistanceRateLimiter_T, _DistanceRateLimiter_R, -_DistanceRateLimiter_R, 0.0);
  ChangeYawRateLimiter.init(_YawRateLimiter_T, _YawRateLimiter_R, -_YawRateLimiter_R, 0.0);

  XRateLimiter.init(_SetpointRateLimiter_T, _SetpointRateLimiter_R, -_SetpointRateLimiter_R, 0.0);
  XRateLimiter.initialCondition(_currCarrotReference.transforms[0].translation.x, _currCarrotReference.transforms[0].translation.x);
  YRateLimiter.init(_SetpointRateLimiter_T, _SetpointRateLimiter_R, -_SetpointRateLimiter_R, 0.0);
  YRateLimiter.initialCondition(_currCarrotReference.transforms[0].translation.y, _currCarrotReference.transforms[0].translation.y);
  ZRateLimiter.init(_SetpointRateLimiter_T, _SetpointRateLimiter_R, -_SetpointRateLimiter_R, 0.0);
  ZRateLimiter.initialCondition(_currCarrotReference.transforms[0].translation.z, _currCarrotReference.transforms[0].translation.z);



  ROS_INFO("Initializing rate limiters");

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
  k = configMsg.filter_k;

  _DistanceRateLimiter_R = configMsg.distance_R;
  _DistanceRateLimiter_T = configMsg.distance_T;
  _DistanceRateLimiter_F - configMsg.distance_F;
  _HeightRateLimiter_R = configMsg.height_R;
  _HeightRateLimiter_T = configMsg.height_T;
  _YawRateLimiter_R = configMsg.yaw_R;
  _YawRateLimiter_T = configMsg.yaw_T;

  _SetpointRateLimiter_R = configMsg.main_R;
  _SetpointRateLimiter_T = configMsg.main_T;
}

void VisualServo::odomCb(const nav_msgs::OdometryConstPtr& odom) {

    _qx = odom->pose.pose.orientation.x;
    _qy = odom->pose.pose.orientation.y;
    _qz = odom->pose.pose.orientation.z;
    _qw = odom->pose.pose.orientation.w;

    _uavRoll = atan2( 2*(_qw * _qx + _qy * _qz), 1 - 2 * (_qx*_qx + _qy*_qy) );
    _uavPitch = asin( 2*(_qw*_qy - _qx*_qz) );

}

void VisualServo::CarrotReferenceCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint msg){
  _currCarrotReference = msg;
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
    else {
      _uavPos[0] = _setpointPosition[0];
    }

    if (msg.y == 0.0) {
        _y_frozen = true;
    }
    if (!_y_frozen) {
        _uavPos[1] = msg.y;
    }
    else {
      _uavPos[1] = _setpointPosition[1];
    }

    if (msg.z == 0.0) {
      _z_frozen = true;
    }
    if (!_z_frozen) {
      _uavPos[2] = msg.z;
    }
    else {
      _uavPos[2] = _setpointPosition[2];
    }

    if (msg.yaw == 0.0) {
        _yaw_frozen = true;
    }
    if (!_yaw_frozen) {
        _uavYaw = msg.yaw;
    }
    else {
      _uavYaw = _setpointYaw;
    }
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

void VisualServo::updateRateLimiters(){
  MoveForwardRateLimiter.setR(_DistanceRateLimiter_R);
  MoveForwardRateLimiter.setF(_DistanceRateLimiter_F);
  MoveForwardRateLimiter.setSampleTime(_DistanceRateLimiter_T);

  MoveUpRateLimiter.setR(_HeightRateLimiter_R);
  MoveUpRateLimiter.setF(- _HeightRateLimiter_R);
  MoveUpRateLimiter.setSampleTime(_HeightRateLimiter_T);

  ChangeYawRateLimiter.setR(_YawRateLimiter_R);
  ChangeYawRateLimiter.setF(- _YawRateLimiter_R);
  ChangeYawRateLimiter.setSampleTime(_YawRateLimiter_T);

  XRateLimiter.setR(_SetpointRateLimiter_R);
  XRateLimiter.setF(- _SetpointRateLimiter_R);
  XRateLimiter.setSampleTime(_SetpointRateLimiter_T);

  YRateLimiter.setR(_SetpointRateLimiter_R);
  YRateLimiter.setF(- _SetpointRateLimiter_R);
  YRateLimiter.setSampleTime(_SetpointRateLimiter_T);

  ZRateLimiter.setR(_SetpointRateLimiter_R);
  ZRateLimiter.setF(- _SetpointRateLimiter_R);
  ZRateLimiter.setSampleTime(_SetpointRateLimiter_T);
}

void VisualServo::updateSetpoint() {
  double move_forward = 0.0;
  double move_forward_limited = 0.0;
  double move_left = 0.0;
  double move_up = 0.0;
  double move_up_limited = 0.0;
  double change_yaw = 0.0;
  double change_yaw_limited = 0.0;

  double _newXSetpoint = 0.0, _newYSetpoint = 0.0, _newZSetpoint = 0.0;

  if (!_x_frozen) move_left = _x_axis_PID.compute(_offset_x, _error_x, 1 / _rate);
  // PT1
  // if (!_y_frozen){
  //   move_forward  = _y_axis_PID.compute(_offset_y, _error_y, 1 / _rate);
  //   move_forward_filtered = k * move_forward + (1 - k) * move_forward_old;
  //   move_forward_old = move_forward_filtered;
  // } 
  // Rate Limiter
  if (!_y_frozen){
    move_forward  = _y_axis_PID.compute(_offset_y, _error_y, 1 / _rate);

    // MoveForwardRateLimiter.setInput(move_forward);
    // move_forward_limited = MoveForwardRateLimiter.getData();
  } 

  // if (!_z_frozen) move_up = _z_axis_PID.compute(_offset_z, _error_z, 1 / _rate);

  if (!_z_frozen){
    move_up = _z_axis_PID.compute(_offset_z, _error_z, 1 / _rate);

    // MoveUpRateLimiter.setInput(move_up);
    // move_up_limited = MoveUpRateLimiter.getData();
  } 

  // if (!_yaw_frozen) change_yaw = _yaw_PID.compute(0, _error_yaw, 1 / _rate);

  if (!_yaw_frozen) {
    change_yaw = _yaw_PID.compute(0, _error_yaw, 1 / _rate);

    ChangeYawRateLimiter.setInput(change_yaw);
    change_yaw_limited = ChangeYawRateLimiter.getData();

  }

  _newXSetpoint = _uavPos[0] + move_forward * cos(_uavYaw + _yaw_added_offset) - move_left * sin(_uavYaw + _yaw_added_offset);
  _newYSetpoint = _uavPos[1] + move_forward * sin(_uavYaw + _yaw_added_offset) + move_left * cos(_uavYaw + _yaw_added_offset);
  _newZSetpoint = _uavPos[2] + move_up;

  XRateLimiter.setInput(_newXSetpoint);
  YRateLimiter.setInput(_newYSetpoint);
  ZRateLimiter.setInput(_newZSetpoint);

  // XRateLimiter.initialCondition(_newXSetpoint, _setpointPosition[0]);
  // YRateLimiter.initialCondition(_newYSetpoint, _setpointPosition[1]);
  // ZRateLimiter.initialCondition(_newZSetpoint, _setpointPosition[2]);

  _setpointPosition[0] = XRateLimiter.getData();
  _setpointPosition[1] = YRateLimiter.getData();
  _setpointPosition[2] = ZRateLimiter.getData();

  // _setpointPosition[0] = _uavPos[0] + move_forward_limited * cos(_uavYaw + _yaw_added_offset);
  // _setpointPosition[0] -= move_left * sin(_uavYaw + _yaw_added_offset);
  // _setpointPosition[1] = _uavPos[1] + move_forward_limited * sin(_uavYaw + _yaw_added_offset);
  // _setpointPosition[1] += move_left * cos(_uavYaw + _yaw_added_offset);
  // _setpointPosition[2] = _uavPos[2] + move_up_limited;

  _setpointYaw = _uavYaw + change_yaw_limited;

  // MSGS
  _moveLeftMsg.data = move_left;
  _changeYawMsg.data = change_yaw;
  _moveUpMsg.data = -move_up;
  _moveForwardMsg.data = move_forward;

  _moveForwardLimitedMsg.data = move_forward_limited;
  _moveUpLimitedMsg.data = -move_up_limited;
  _changeYawLimitedMsg.data = change_yaw_limited;

  // PUB
  _pubMoveLeft.publish(_moveLeftMsg);
  _pubChangeYaw.publish(_changeYawMsg);
  _pubMoveForward.publish(_moveForwardMsg);
  _pubMoveUp.publish(_moveUpMsg);
  _moveLeftMsg.data = _setpointPosition[0]- _uavPos[0];
  _pubXAdd.publish(_moveLeftMsg);
  _moveLeftMsg.data = _setpointPosition[1]- _uavPos[1];
  _pubYAdd.publish(_moveLeftMsg);

  _pubMoveForwardLimited.publish(_moveForwardLimitedMsg);
  _pubMoveUpLimited.publish(_moveUpLimitedMsg);
  _pubChangeYawLimited.publish(_changeYawLimitedMsg);


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
    visualServoRefObj.updateRateLimiters();
    visualServoRefObj.updateSetpoint();
    if (visualServoRefObj.isVisualServoEnabled()) {
      visualServoRefObj.publishNewSetpoint();
    }
    visualServoRefObj.publishStatus();
    loopRate.sleep();
  }
}
} // namespace uav_reference