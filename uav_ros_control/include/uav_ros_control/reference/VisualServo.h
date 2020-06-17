//
// Created by robert on 20. 09. 2019..
//

#ifndef UAV_ROS_CONTROL_VISUALSERVO_H
#define UAV_ROS_CONTROL_VISUALSERVO_H

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>

#include <dynamic_reconfigure/server.h>
#include <uav_ros_control/VisualServoParametersConfig.h>
#include <uav_ros_control/control/PID.h>
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/reference/RateLimiter.h>


namespace uav_reference {
/**
 * Name of dynamic reconfigure node.
 */
#define VISUAL_SERVO_DYN_RECONF "visual_servo_config"

 /**
  * Publish UAV reference based on the error reported by the drone's color filter.
  * The reference is published in the global coordinate system.
  **/
  class VisualServo {
   public:
     /**
      * Default constructor.
      */
     VisualServo(ros::NodeHandle&);
     virtual ~VisualServo();

     /**
      * Update position setpoint.
      */
     void updateSetpoint();

     /**
      * Update Rate Limiters.
      */
     void updateRateLimiters();

     /**
      * Publish new setpoint as MultiDOFJointTrajectoryPoint
      */
     void publishNewSetpoint();
     bool isVisualServoEnabled();
     void publishStatus();
     void setRate(double new_rate) {_rate =  new_rate;}
     void initializeParameters(ros::NodeHandle&);
   private:

      /**
       * Callback function for StartVisualServo service
       */
      bool startVisualServoServiceCb(std_srvs::SetBool::Request& request,
          std_srvs::SetBool::Response& response);

      /**
        * Odometry callback function for extracting the UAV's pose.
        */
      void odomCb(const nav_msgs::OdometryConstPtr&);
      void CarrotReferenceCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint msg);


      /**
       * Callback functions for the visual servo process values.
       *
       * Use xErrorCb and yErrorCb when the camera is facing the floor (brick laying scenario).
       */
      void xErrorCb(const std_msgs::Float32&);
      void yErrorCb(const std_msgs::Float32&);
      void zErrorCb(const std_msgs::Float32&);
      void yawErrorCb(const std_msgs::Float32&);
      void VisualServoProcessValuesCb(const uav_ros_control_msgs::VisualServoProcessValues&);
      void xOffsetCb(const std_msgs::Float32&);
      void yOffsetCb(const std_msgs::Float32&);
      void zOffsetCb(const std_msgs::Float32&);

      // X and Y axes of the image coordinate frame.
      PID _x_axis_PID{"x-axis"}, _y_axis_PID{"y-axis"}, _z_axis_PID{"z-axis"}, _yaw_PID{"yaw"};

      int _n_contours = 0;
      std::array<double, 3> _uavPos{0.0, 0.0, 0.0};
      std::array<double, 3> _setpointPosition{0.0, 0.0, 0.0};
      double _error_x = 0, _error_y = 0, _error_z = 0, _error_yaw = 0, _offset_x = 0;
      double _offset_y = 0, _offset_z = 0,  _deadzone_x = 0, _deadzone_y = 0, _deadzone_z = 0, _deadzone_yaw = 0;
      double _uavYaw, _uavRoll, _uavPitch, _setpointYaw;
      double _rate, _camera_fov, _yaw_added_offset;
      double _qx, _qy, _qz, _qw;

      bool _visualServoEnabled = false,  _compensate_roll_and_pitch = false;
      bool _x_frozen = false, _y_frozen = false, _z_frozen = false,  _yaw_frozen = false;

      trajectory_msgs::MultiDOFJointTrajectoryPoint _currCarrotReference;

      /** Publishers */
      ros::Publisher _pubNewSetpoint;
      trajectory_msgs::MultiDOFJointTrajectoryPoint _new_point;

      // Status topic
      ros::Publisher _pubIsEnabledTopic;
      std_msgs::Bool _boolMsg;

      // Topics for direct rotor control
      ros::Publisher _pubMoveLeft, _pubMoveForward, _pubMoveUp;
      std_msgs::Float32 _moveLeftMsg, _moveForwardMsg, _moveUpMsg;

      // Position VS (+ RL) output
      ros::Publisher _pubPositionUnlimited, _pubPositionLimited;
      geometry_msgs::Point _positionUnlimitedMsg, _positionLimitedMsg;

      // Yaw VS (+ RL) output
      ros::Publisher _pubChangeYawUnlimited, _pubChangeYawLimited;
      std_msgs::Float32 _yawUnlimitedMsg, _yawLimitedMsg;

      // Topics for debugging
      ros::Publisher _pubChangeYawDebug, _pubYawErrorDebug, _pubXAdd, _pubYAdd;
      std_msgs::Float32 _floatMsg;

      // Brick errors publisher
      ros::Publisher _pubXError, _pubYError, _pubZError;

      /** Subscribers */
      ros::Subscriber _subOdom, _subImu, _subCarrotTrajectory;
      ros::Subscriber _subXError, _subYError, _subZError, _subYawError, _subNContours;
      ros::Subscriber _subVisualServoProcessValuesMsg;
      ros::Subscriber _subXOffset, _subYOffset, _subZOffset;

      uav_ros_control_msgs::VisualServoProcessValues VisualServoProcessValuesMsg;

      /** Services */
      ros::ServiceServer _serviceStartVisualServo;
      std_srvs::SetBool::Request _setBoolRequest;
      std_srvs::SetBool::Response _setBoolResponse;

      // Rate Limiters
      bool _enableRateLimiters;
      
      RateLimiter ChangeYawRateLimiter;
      float _YawRateLimiter_R, _YawRateLimiter_T;

      RateLimiter XRateLimiter, YRateLimiter, ZRateLimiter;
      float _SetpointRateLimiter_T, _SetpointRateLimiter_R;

      void visualServoParamsCb(
          uav_ros_control::VisualServoParametersConfig& configMsg, uint32_t level);

      /** Define Dynamic Reconfigure parameters **/
      boost::recursive_mutex _VSConfigMutex;
      dynamic_reconfigure::
      Server<uav_ros_control::VisualServoParametersConfig>
          _VSConfigServer {_VSConfigMutex, ros::NodeHandle(VISUAL_SERVO_DYN_RECONF)};
      dynamic_reconfigure::
      Server<uav_ros_control::VisualServoParametersConfig>::CallbackType
          _VSParamCallback;
  };

  void runDefault(VisualServo& cc, ros::NodeHandle& nh);
}

#endif //UAV_ROS_CONTROL_VISUALSERVO_H

