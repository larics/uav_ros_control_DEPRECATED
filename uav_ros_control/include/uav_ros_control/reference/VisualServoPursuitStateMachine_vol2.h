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
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_control/VisualServoPursuitParametersConfig.h>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <uav_ros_control/GenerateSearch.h>
#include <uav_ros_control/GenerateInterception.h>
#include <uav_ros_control/reference/Global2Local.h>

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
#define PARAM_SETTLE_THRESHOLD          "pursuit/state_machine/settle_threshold"
#define PARAM_GPS_NUM_POINTS            "pursuit/state_machine/gps_num_points"
#define PARAM_COLLECT_TIMEOUT           "pursuit/state_machine/collect_timeout"
#define PARAM_AFTER_TAKEOFF_HEIGHT      "pursuit/state_machine/after_takeoff_height"
#define PARAM_LAND_LAT                  "pursuit/state_machine/gps_land/lat"
#define PARAM_LAND_LON                  "pursuit/state_machine/gps_land/lon"
#define PARAM_LAND_HEIGHT               "pursuit/state_machine/gps_land/height"

enum PursuitState {
    OFF,
    GPS_POSITION,
    COLLECT_POINTS,
    INTERCEPTION,
    WAIT,
    LAND,
    END,
    SETTLE,
    SEARCH,
    UAV_FOLLOWING,
    BALL_GRASPING
};

class PursuitStateMachine
{

public:

PursuitStateMachine(ros::NodeHandle& nh):
_global2Local(nh)
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
    _subCarrotReference = nh.subscribe("carrot/trajectory", 1, &uav_reference::PursuitStateMachine::carrotReferenceCb, this);
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
    _subHausdorffEstimator = nh.subscribe("target_uav/setpoint_estimated", 1, &uav_reference::PursuitStateMachine::hausdorffEstimatorCb, this);
    _subCurrentEstimatedTargetPoint = nh.subscribe("target_uav/position_estimated", 1, &uav_reference::PursuitStateMachine::currentEstimatedTargetPointCb, this);
    _subBackupEstimator = nh.subscribe("target_uav/backup_setpoint_estimated", 1, &uav_reference::PursuitStateMachine::backupEstimatorCb, this);
    // _subEstimatorStatus = nh.subscribe("figure8_state", 1, &uav_reference::PursuitStateMachine::estimatorStatusCb, this);
    _subToppStatus = nh.subscribe("topp/status", 1, &uav_reference::PursuitStateMachine::toppStatusCb, this);

    /* Takeoff done*/
    _subReadyForPursuit = nh.subscribe("ready_for_exploration", 1, &uav_reference::PursuitStateMachine::readyForPursuitCb, this);
    _pubInputTrajectory = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("topp/input/trajectory", 1);

    _pubEstimatorStart = nh.advertise<std_msgs::Bool>("sm_pursuit/start_estimator", 1);


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

    // Client for LAND service
    _landClient = nh.serviceClient<std_srvs::SetBool>("land");

    // Client for Figure-8-estimator reset
    _estimatorResetCaller = nh.serviceClient<std_srvs::Empty>("reset_figure8_estimator");
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

void hausdorffEstimatorCb(geometry_msgs::PoseStamped msg){
    _hausdorffPoint = msg;
    _hausdorffIn = true;
    // _hausdorffPoint.pose.position.z += _interceptionZOffset; /////////////////////////// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // _interceptionActivated = true;
}
void backupEstimatorCb(geometry_msgs::PoseStamped msg){
    _backupPoint = msg;
    _backupIn = true;
}

void currentEstimatedTargetPointCb(geometry_msgs::PointStamped msg){
    _currentEstimatedTargetPoint = msg;
    _searchEstimated = true;
    _estimatedEverReceived = true;

}

void estimatorStatusCb(std_msgs::Bool msg){
    _hausdorffStatus = msg.data;
}

void toppStatusCb(std_msgs::Bool msg){
    _toppStatus = msg;
}

void readyForPursuitCb(std_msgs::Bool msg){
    _readyForPursuit = msg.data;

    // if(_readyForPursuit)
    //     _gpsPursuitActivated = true;
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
    _settleThreshold = configMsg.settle_threshold;
    _numOfGPSPoints = configMsg.gps_num_points;
    _collectTimeout = configMsg.collect_timeout;
    _afterTakeOffHeight = configMsg.after_takeoff_height;
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
    config.settle_threshold = _settleThreshold;
    config.gps_num_points = _numOfGPSPoints;
    config.collect_timeout = _collectTimeout;
    config.after_takeoff_height = _afterTakeOffHeight;
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
    && nh.getParam(PARAM_YAW_ERROR_DEADZONE, _yawDeadZoneThreshold)
    && nh.getParam(PARAM_SETTLE_THRESHOLD, _settleThreshold)
    && nh.getParam(PARAM_GPS_NUM_POINTS, _numOfGPSPoints)
    && nh.getParam(PARAM_COLLECT_TIMEOUT, _collectTimeout)
    && nh.getParam(PARAM_AFTER_TAKEOFF_HEIGHT, _afterTakeOffHeight)
    && nh.getParam(PARAM_LAND_LAT, _landLat)
    && nh.getParam(PARAM_LAND_LON, _landLon)
    && nh.getParam(PARAM_LAND_HEIGHT, _landHeight);
    // Tip: Define parameter name at the top of the file

    // Waiting for home position
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    // Get GPS points from yaml
    double t_lat, t_lon, t_alt, t_hdg;
    std::string s;
    for (int i = 0; i < _numOfGPSPoints ; i++){
        s = std::to_string(i+1);
        initialized = initialized
        && nh.getParam("pursuit/gps_points/" + s + "/lat", t_lat)
        && nh.getParam("pursuit/gps_points/" + s + "/lon", t_lon)
        && nh.getParam("pursuit/gps_points/" + s + "/alt", t_alt)
        && nh.getParam("pursuit/gps_points/" + s + "/hdg", t_hdg);

        // std::cout << t_lat << " " << t_lon << " "  << t_alt << std::endl;

        // Transform global to local
        // Eigen::Vector3d localPoint = _global2Local.toLocal(t_lat, t_lon, t_alt);
        Eigen::Vector3d localPoint = _global2Local.toLocal(t_lat, t_lon, t_alt, true);


        geometry_msgs::Transform tmp_transform;
        tmp_transform.translation.x = localPoint.x();
        tmp_transform.translation.y = localPoint.y();
        tmp_transform.translation.z = localPoint.z();

        // Convert Yaw to Quaternion
        tf2::Quaternion tmp_q;
        tmp_q.setRPY(0, 0, t_hdg);

        tmp_transform.rotation.x = tmp_q.x();
        tmp_transform.rotation.y = tmp_q.y();
        tmp_transform.rotation.z = tmp_q.z();
        tmp_transform.rotation.w = tmp_q.w();

        trajectory_msgs::MultiDOFJointTrajectoryPoint tmp_point;
        tmp_point.transforms.push_back(tmp_transform);

        _gpsPoints.points.push_back(tmp_point);

    }

    _toppStatus.data = false;
    _inferenceEnabled.data = true;


    ROS_INFO("Node rate: %.2f", _rate);
    if (!initialized)
	{
		ROS_FATAL("PursuitStateMachine::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("PursuitStateMachine parameters not properly initialized.");
	}
}

void requestHeightTrajectory(){
    trajectory_msgs::MultiDOFJointTrajectory temp_trajectory;

    geometry_msgs::Transform tmp_transform;
    // Odometry
    // tmp_transform.translation.x = _currOdom.pose.pose.position.x;
    // tmp_transform.translation.y = _currOdom.pose.pose.position.y;
    // tmp_transform.translation.z = _currOdom.pose.pose.position.z;

    // tmp_transform.rotation.x = _currOdom.pose.pose.orientation.x;
    // tmp_transform.rotation.y = _currOdom.pose.pose.orientation.y;
    // tmp_transform.rotation.z = _currOdom.pose.pose.orientation.z;
    // tmp_transform.rotation.w = _currOdom.pose.pose.orientation.w;

    // Reference
    tmp_transform.translation.x = _currCarrotReference.transforms[0].translation.x;
    tmp_transform.translation.y = _currCarrotReference.transforms[0].translation.y;
    tmp_transform.translation.z = _currCarrotReference.transforms[0].translation.z;

    tmp_transform.rotation.x = _currCarrotReference.transforms[0].rotation.x;
    tmp_transform.rotation.y = _currCarrotReference.transforms[0].rotation.y;
    tmp_transform.rotation.z = _currCarrotReference.transforms[0].rotation.z;
    tmp_transform.rotation.w = _currCarrotReference.transforms[0].rotation.w;

    trajectory_msgs::MultiDOFJointTrajectoryPoint starting_point;
    starting_point.transforms.push_back(tmp_transform);

    // Keep x and y coordinates, just increase height after takeoff
    temp_trajectory.points.push_back(starting_point);

    tmp_transform.translation.z = _afterTakeOffHeight;

    trajectory_msgs::MultiDOFJointTrajectoryPoint height_point;
    height_point.transforms.push_back(tmp_transform);

    temp_trajectory.points.push_back(height_point);
    if ( _afterTakeOffHeight > _currCarrotReference.transforms[0].translation.z && abs(_afterTakeOffHeight - _currCarrotReference.transforms[0].translation.z) > 0.5){
        ROS_INFO("PursuitSM::updateStatus - Going to starting height.");
        _pubInputTrajectory.publish(temp_trajectory);
    }
    else
        ROS_INFO("PursuitSM::updateStatus - Already near starting height.");


}

void requestLandTrajectory(){
    trajectory_msgs::MultiDOFJointTrajectory t_land_trajectory;

    geometry_msgs::Transform tmp_transform;

    // Reference
    tmp_transform.translation.x = _currCarrotReference.transforms[0].translation.x;
    tmp_transform.translation.y = _currCarrotReference.transforms[0].translation.y;
    tmp_transform.translation.z = _currCarrotReference.transforms[0].translation.z;

    tmp_transform.rotation.x = _currCarrotReference.transforms[0].rotation.x;
    tmp_transform.rotation.y = _currCarrotReference.transforms[0].rotation.y;
    tmp_transform.rotation.z = _currCarrotReference.transforms[0].rotation.z;
    tmp_transform.rotation.w = _currCarrotReference.transforms[0].rotation.w;

    trajectory_msgs::MultiDOFJointTrajectoryPoint starting_point;
    starting_point.transforms.push_back(tmp_transform);

    t_land_trajectory.points.push_back(starting_point);

    double t_hdg = 0.0;

    // Add GPS landing point
    Eigen::Vector3d localPoint = _global2Local.toLocal(_landLat, _landLon, _landHeight, true);

    tmp_transform.translation.x = localPoint.x();
    tmp_transform.translation.y = localPoint.y();
    tmp_transform.translation.z = localPoint.z();

    // Convert Yaw to Quaternion
    tf2::Quaternion tmp_q;
    tmp_q.setRPY(0, 0, t_hdg);

    tmp_transform.rotation.x = tmp_q.x();
    tmp_transform.rotation.y = tmp_q.y();
    tmp_transform.rotation.z = tmp_q.z();
    tmp_transform.rotation.w = tmp_q.w();

    trajectory_msgs::MultiDOFJointTrajectoryPoint land_point;
    land_point.transforms.push_back(tmp_transform);

    t_land_trajectory.points.push_back(land_point);

    _pubInputTrajectory.publish(t_land_trajectory);

}



void requestGPSPointTrajectory(){
    trajectory_msgs::MultiDOFJointTrajectory t_gps_trajectory;

    geometry_msgs::Transform tmp_transform;
    // Odometry
    // tmp_transform.translation.x = _currOdom.pose.pose.position.x;
    // tmp_transform.translation.y = _currOdom.pose.pose.position.y;
    // tmp_transform.translation.z = _currOdom.pose.pose.position.z;

    // tmp_transform.rotation.x = _currOdom.pose.pose.orientation.x;
    // tmp_transform.rotation.y = _currOdom.pose.pose.orientation.y;
    // tmp_transform.rotation.z = _currOdom.pose.pose.orientation.z;
    // tmp_transform.rotation.w = _currOdom.pose.pose.orientation.w;

    // Reference
    tmp_transform.translation.x = _currCarrotReference.transforms[0].translation.x;
    tmp_transform.translation.y = _currCarrotReference.transforms[0].translation.y;
    tmp_transform.translation.z = _currCarrotReference.transforms[0].translation.z;

    tmp_transform.rotation.x = _currCarrotReference.transforms[0].rotation.x;
    tmp_transform.rotation.y = _currCarrotReference.transforms[0].rotation.y;
    tmp_transform.rotation.z = _currCarrotReference.transforms[0].rotation.z;
    tmp_transform.rotation.w = _currCarrotReference.transforms[0].rotation.w;

    trajectory_msgs::MultiDOFJointTrajectoryPoint starting_point;
    starting_point.transforms.push_back(tmp_transform);

    t_gps_trajectory.points.push_back(starting_point);

    // Add final point 
    trajectory_msgs::MultiDOFJointTrajectoryPoint t_final_point;
    t_final_point = _gpsPoints.points[_gpsPointIndex];

    t_gps_trajectory.points.push_back(t_final_point);

    _pubInputTrajectory.publish(t_gps_trajectory);

}

void requestSearchTrajectory(){
    ROS_INFO("PursuitSM::update status - requesting search trajectory.");
    uav_ros_control::GenerateSearch srv;

    if (_searchEstimated){
        ROS_FATAL("PursuitSM::updateStatus - ESTIMATED search requested.");
        srv.request.type = "estimated";
    }
    else{
        ROS_FATAL("PursuitSM::updateStatus - SCAN search requested.");
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

void requestInterceptionTrajectory(geometry_msgs::PoseStamped msg){
    trajectory_msgs::MultiDOFJointTrajectory t_interception_trajectory;

    geometry_msgs::Transform tmp_transform;

    // Reference
    tmp_transform.translation.x = _currCarrotReference.transforms[0].translation.x;
    tmp_transform.translation.y = _currCarrotReference.transforms[0].translation.y;
    tmp_transform.translation.z = _currCarrotReference.transforms[0].translation.z;

    tmp_transform.rotation.x = _currCarrotReference.transforms[0].rotation.x;
    tmp_transform.rotation.y = _currCarrotReference.transforms[0].rotation.y;
    tmp_transform.rotation.z = _currCarrotReference.transforms[0].rotation.z;
    tmp_transform.rotation.w = _currCarrotReference.transforms[0].rotation.w;

    trajectory_msgs::MultiDOFJointTrajectoryPoint starting_point;
    starting_point.transforms.push_back(tmp_transform);

    t_interception_trajectory.points.push_back(starting_point);

    // Interception point
    tmp_transform.translation.x = msg.pose.position.x;
    tmp_transform.translation.y = msg.pose.position.y;
    tmp_transform.translation.z = msg.pose.position.z;

    tmp_transform.rotation.x = msg.pose.orientation.x;
    tmp_transform.rotation.y = msg.pose.orientation.y;
    tmp_transform.rotation.z = msg.pose.orientation.z;
    tmp_transform.rotation.w = msg.pose.orientation.w;

    trajectory_msgs::MultiDOFJointTrajectoryPoint final_point;
    final_point.transforms.push_back(tmp_transform);

    t_interception_trajectory.points.push_back(final_point);

    _pubInputTrajectory.publish(t_interception_trajectory);

}

// void requestInterceptionTrajectory(){
//     ROS_INFO("PursuitSM::update status - requesting interception trajectory.");
//     uav_ros_control::GenerateInterception srv;
//     srv.request.interception_point = _hausdorffPoint;

//     if(!_interceptionTrajectoryClientCaller.call(srv)){
//         ROS_FATAL("PursuitSM::updateStatus - interception trajectory not generated.");
//         ROS_INFO("PursuitSM::updateStatus - OFF state activated.");
//         _currentState = PursuitState::OFF;
//         return;
//     }

//     if (srv.response.success){
//         ROS_INFO("PursuitSM::updateStatus - interception trajectory successfully generated.");
//     }

// }

geometry_msgs::PoseStamped point2pose(geometry_msgs::PointStamped t_msg){
    geometry_msgs::PoseStamped tmp_pose;

    tmp_pose.pose.position.x = t_msg.point.x;
    tmp_pose.pose.position.y = t_msg.point.y;
    tmp_pose.pose.position.z = t_msg.point.z - 2.0;

    // Convert Yaw to Quaternion
    tf2::Quaternion tmp_q;
    tmp_q.setRPY(0, 0, 4.1015237422); // 235 deg

    tmp_pose.pose.orientation.x = tmp_q.x();
    tmp_pose.pose.orientation.y = tmp_q.y();
    tmp_pose.pose.orientation.z = tmp_q.z();
    tmp_pose.pose.orientation.w = tmp_q.w();

    return tmp_pose;
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
    checkDetection();
    // If visual servo is inactive, deactivate state machine
    // Visual servo can be inactive in state OFF and SEARCH.
    if (_challengeTime > _challengeTimeout && _currentState != PursuitState::LAND && _currentState != PursuitState::END){
        ROS_INFO("PursuitSM::updateStatus - LAND state activated. Going to LAND GPS point.");
        _searchTrajectoryFlag.data = true;
        requestLandTrajectory();
        ros::Duration(1.0).sleep();
        _currentState = PursuitState::LAND;
        return;
    }

    if (_currentState == PursuitState::LAND && !_toppStatus.data){
        std_srvs::SetBool::Request landRequest;
        std_srvs::SetBool::Response landResponse;
        ros::Duration(2.0).sleep();
        ROS_WARN("PursuitSM::updateStatus - Calling LAND service.");
        landRequest.data = true;
        if (!_landClient.call(landRequest, landResponse)) {
            ROS_FATAL("MasterPickupControl::land_uav - call to LAND service failed");
            return;
        }

        if (!landResponse.success) {
            ROS_FATAL("MasterPickupControl::land_uav - LAND request failed.");
            return;
        }
        ROS_INFO("MasterPickupControl::land_uav - LAND request succesfful."); 
        _currentState = PursuitState::END;
        return;
    }

    if (_currentState == PursuitState::WAIT){
        return;
    }

    if (_currentState != PursuitState::OFF && !_gpsPursuitActivated && !_interceptionActivated)
    {
        ROS_WARN("PursuitSM::updateStatus - Visual servo is inactive.");
        _currentState = PursuitState::OFF;
        _collectTime = 0;
        _followingStartTime = 0;
        turnOffVisualServo();
        _searchTrajectoryFlag.data = false;
        _inferenceEnabled.data = true;
        ROS_WARN("PursuitSM::updateStatus - OFF State activated.");
        return;
    }

    if( _currentState == OFF && !_gpsPursuitActivated && _readyForPursuit && !_gpsPursuitFinished && !_interceptionActivated){
        ROS_INFO("PursuitSM::updateStatus - TakeOff successful. Increase height.");
        _searchTrajectoryFlag.data = true;
        requestHeightTrajectory();
        ros::Duration(1.0).sleep();
        _gpsPursuitActivated = true;
        return;

    }

    if (_currentState == PursuitState::OFF && _gpsPursuitActivated && !_toppStatus.data){
        // Clear trajectory
        _searchTrajectoryFlag.data = false;
        _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);

        ROS_INFO("PursuitSM::updateStatus - GPS_POSITION status activated.");
        ROS_WARN("PursuitSM::updateStatus - Going to GPS position num : %d", _gpsPointIndex + 1);

        _searchTrajectoryFlag.data = true;
        _currentState = PursuitState::GPS_POSITION;
        return;
    }

    if (_currentState == PursuitState::GPS_POSITION && _gpsPursuitActivated){
        ROS_WARN("PursuitSM::updateStatus - Trajectory to watching point.");
        ROS_INFO("PursuitSM::updateStatus - COLLECT_POINTS status activated.");

        requestGPSPointTrajectory();
        ros::Duration(1.0).sleep();
        _gpsPointIndex = _gpsPointIndex + 1;
        _currentState = PursuitState::COLLECT_POINTS;
        _collectTime = 0;
        return;
    }

    if(_currentState == PursuitState::COLLECT_POINTS && !_gpsPursuitFinished){

        if (!_toppStatus.data){
            _collectTime += 1/_rate;

            if (_collectTime > _collectTimeout){
                // Prepare backup points
                if (_gpsPointIndex == 4){
                    _point2 = _currentEstimatedTargetPoint;
                    _point2In = true;
                    std::cout << _point2 << std::endl;
                }

                // Note: GPS Point index starts from 0
                if (_gpsPointIndex < _numOfGPSPoints){
                    ROS_INFO("PursuitSM::updateStatus - I finished collecting trajectory points in %d / %d GPS point.", _gpsPointIndex, _numOfGPSPoints);
                    _currentState = PursuitState::OFF;
                    _searchTrajectoryFlag.data = false;
                }
                else{
                    ROS_INFO("PursuitSM::updateStatus - Finished collecting points in last GPS point");
                    _currentState = PursuitState::INTERCEPTION;
                    _gpsPursuitActivated = false;
                    _gpsPursuitFinished = true;
                    _interceptionActivated = true;
                    _searchTrajectoryFlag.data = false;
                    return;
                }
            }
        }
        else if ( _gpsPointIndex == 1){
            ROS_FATAL_ONCE("PursuitSM::updateStatus - Start figure-8-estimator.");
            std_msgs::Bool start_msg;
            start_msg.data = true;
            _pubEstimatorStart.publish(start_msg);
        }
        return;
    }

    if ( _currentState == PursuitState::INTERCEPTION && !_interceptionSent){
        ROS_INFO("PursuitSM::updateStatus - INTERCEPTION state.");
        _searchTrajectoryFlag.data = true;
        _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
        if(_hausdorffStatus && _hausdorffIn){
            // Estimated lemniscate point
            ROS_WARN("PursuitSM::updateStatus - Interception at lemniscate.");
            requestInterceptionTrajectory(_hausdorffPoint);
        }
        else if (_backupIn){
            ROS_WARN("PursuitSM::updateStatus - Interception at backup point.");
            requestInterceptionTrajectory(_backupPoint);
        }
        else if (_point2In && _estimatedEverReceived){
            ROS_WARN("PursuitSM::updateStatus - Interception at random estimated point around num 2.");
            requestInterceptionTrajectory(point2pose(_point2));
        }
        else{
            ROS_FATAL("PursuitSM::updateStatus - Never receieved point. Go to MANUAL !!!!!");

        }
        ROS_INFO("PursuitSM::updateStatus - go to WAIT.");
        _currentState == PursuitState::WAIT;
        _interceptionSent = true;
        return;
    }


    // // Request search trajectory
    // if (_currentState == PursuitState::OFF && (!_start_following_uav || !_isDetectionActive) && _pursuitActivated){
    //     ROS_WARN("PursuitSM::updateStatus - SEARCH state activated.");
    //     turnOffVisualServo();
    //     _currentState = PursuitState::SEARCH;
    //     _searchTrajectoryFlag.data = true;
    //     _inferenceEnabled.data = true;
    //     _followingStartTime = 0;

    //     return;
    // }

    // if (_currentState == PursuitState::SEARCH && (!_start_following_uav || !_isDetectionActive) && _pursuitActivated){
    //     if (!_toppStatus.data){
    //         requestSearchTrajectory();
    //         ros::Duration(0.5).sleep();
    //     }
    //     _followingStartTime = 0;
    //     return;
    // }

    // // Activate Pursuit algorithm when detection is confident.
    // if ((_currentState == PursuitState::OFF || _currentState == PursuitState::SEARCH || _currentState == PursuitState::SETTLE)
    //  && _start_following_uav && _isDetectionActive && _pursuitActivated && !_interceptionActivated)
    // {
    //     ROS_INFO("PursuitSM::updateStatus - Starting visual servo for UAV following.");
    //     _currentState = PursuitState::UAV_FOLLOWING;
    //     _searchTrajectoryFlag.data = false;
    //     _inferenceEnabled.data = true;
    //     turnOnVisualServo();
    //     _followingStartTime = 1/_rate;

    //     // Comment this out for testing pursposes
    //     //_currHeightReference = 0;
    //     //_currDistanceReference = _uav_distance_offset;

    //     _currDistanceReference = _relativeUAVDistance;

    //     _currHeightReference = _relativeUAVHeight;
    //     _currYawReference = _relativeUAVYaw;

    //     if (isRelativeDistanceNan()){
    //         _currDistanceReference = _maxDistanceReference;
    //         ROS_WARN("PursuitSM::updateStatus - UAV distance is Nan.");
    //     }
    //     else if (!isRelativeDistancePositive()) {
    //         // pass
    //         ROS_FATAL("PursuitSM::updateStatus - UAV distance is negative.");
    //         _currDistanceReference = _uav_distance_offset;
    //     }
        
    //     return;
    // }
    // // Transition to ball following when distance is below certain threshold.
    // // If detection is not confident anymore or visual servo is deactivated or detection node is inactive, turn off UAV following.
    // if (_currentState == PursuitState::UAV_FOLLOWING && (!_start_following_uav || !_pursuitActivated || !_isDetectionActive))
    // {
    //     ROS_WARN("PursuitSM::updateStatus - exiting UAV_FOLLOWING mode.");
    //     ROS_WARN_COND(!_start_following_uav, "PursuitSM::condition - UAV following not confident anymore.");
    //     ROS_WARN_COND(!_pursuitActivated, "PursuitSM::condition - service Pursuit is not active anymore.");
    //     ROS_WARN_COND(!_isDetectionActive, "PursuitSM::condition - detection is inactive.");
    //     _lastCarrotReference = _currCarrotReference;

    //     turnOffVisualServo();
    //     _currentState = PursuitState::SETTLE;
    //     _collectTime = 0;
    //     _followingStartTime = 0;
    //     _inferenceEnabled.data = true;
    //     //requestSearchTrajectory();
    //     ROS_WARN("PursuitSM::updateStatus - SETTLE State activated.");
    //     return;
    // }

    // if (_currentState == PursuitState::SETTLE){
    //     _collectTime += 1/_rate;
    //     // if (_collectTime >= _searchTimeOut){
    //     //     _currentState = PursuitState::OFF;
    //     //     ROS_WARN("PursuitSM::updateStatus - Settle timeout exceeded. Transition to OFF state.");
    //     //     _collectTime = 0.0;
    //     // }
    //     double _settleX, _settleY, _settleZ;

    //     _settleX = abs(_currCarrotReference.transforms[0].translation.x - _currOdom.pose.pose.position.x);
    //     _settleY = abs(_currCarrotReference.transforms[0].translation.y - _currOdom.pose.pose.position.y);
    //     _settleZ = abs(_currCarrotReference.transforms[0].translation.z - _currOdom.pose.pose.position.z);

    //     ROS_INFO("Time: %f X: %f Y: %f Z: %f", _collectTime, _settleX, _settleY, _settleZ);

    //     if (_settleX < _settleThreshold && _settleY < _settleThreshold && _settleZ < _settleThreshold){
    //         _currentState = PursuitState::OFF;
    //         ROS_WARN("PursuitSM::updateStatus - Settle timeout exceeded. Transition to OFF state.");
    //     }


    //     return;
    // }

    // if (_currentState == PursuitState::UAV_FOLLOWING && _interceptionActivated){
    //     ROS_WARN("PursuitSM::updateStatus - Going to SETTLE state to prepare for interception.");
    //     turnOffVisualServo();
    //     _collectTime = 0;
    //     _currentState = PursuitState::SETTLE;
    //     _pursuitActivated = false;
    //     return;
    // }

    // if (_currentState == PursuitState::OFF && _interceptionActivated){
    //     ROS_WARN("PursuitSM::updateStatus - INTERCEPT state activated.");
    //     turnOffVisualServo();
    //     _currentState = PursuitState::INTERCEPTION;
    //     // generate trajectory
    //     requestInterceptionTrajectory();
    //     _searchTrajectoryFlag.data = true;
    //     _inferenceEnabled.data = true;
    //     _followingStartTime = 0;
    //     _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
    //     ros::Duration(2.0).sleep();

    //     return;
    // }

    // if (_currentState == PursuitState::INTERCEPTION && !_toppStatus.data && _interceptionActivated){
    //     ROS_FATAL("PursuitSM::updateStatus - Interception point reached. BALL_GRASPING state activated.");
    //     _currentState = PursuitState::BALL_GRASPING;
    //     _searchTrajectoryFlag.data = false;
    //     _followingStartTime = 0;
    //     ROS_WARN("PursuitSM::updateStatus - CNN inference disabled.");
    //     _inferenceEnabled.data = false;


    //     return;
    // }

    // if (_currentState == PursuitState::UAV_FOLLOWING && _pursuitActivated)
    // {
    //     // Comment this out for testing pursposes
    //     //_currHeightReference = 0;
    //     //_currDistanceReference = _uav_distance_offset;


    //     _inferenceEnabled.data = true;
    //     _followingStartTime += 1/_rate;

    //     _currDistanceReference = _relativeUAVDistance;
    //     _currHeightReference = _relativeUAVHeight;
    //     _currYawReference = _relativeUAVYaw;

    //     if (isRelativeDistanceNan()){
    //         _currDistanceReference = _maxDistanceReference;
    //         ROS_WARN("PursuitSM::updateStatus - UAV distance is Nan.");
    //     }
    //     else if (!isRelativeDistancePositive()) {
    //         // pass
    //         ROS_FATAL("PursuitSM::updateStatus - UAV distance is negative. %f", _relativeUAVDistance);
    //         _currDistanceReference = _uav_distance_offset;
    //     }

    //     return;
    // }
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
        case PursuitState::OFF :
            // _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
            // _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
            // _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
            // _currVisualServoFeed.yaw = util::calculateYaw(
            //     _currOdom.pose.pose.orientation.x,
            //     _currOdom.pose.pose.orientation.y,
            //     _currOdom.pose.pose.orientation.z,
            //     _currOdom.pose.pose.orientation.w);
            if (!_currCarrotReference.transforms.empty()){
                _currVisualServoFeed.x = _currCarrotReference.transforms[0].translation.x;
                _currVisualServoFeed.y = _currCarrotReference.transforms[0].translation.y;
                _currVisualServoFeed.z = _currCarrotReference.transforms[0].translation.z;
                _currVisualServoFeed.yaw = util::calculateYaw(
                    _currCarrotReference.transforms[0].rotation.x,
                    _currCarrotReference.transforms[0].rotation.y,
                    _currCarrotReference.transforms[0].rotation.z,
                    _currCarrotReference.transforms[0].rotation.w);
                break;

            }
        case PursuitState::UAV_FOLLOWING : 
            // if (!_currCarrotReference.transforms.empty() && _followingStartTime < 0.5){
            //     ROS_INFO("Carrot");
            //     _currVisualServoFeed.x = _currCarrotReference.transforms[0].translation.x;
            //     _currVisualServoFeed.y = _currCarrotReference.transforms[0].translation.y;
            //     _currVisualServoFeed.z = _currCarrotReference.transforms[0].translation.z;
            //     _currVisualServoFeed.yaw = util::calculateYaw(
            //         _currCarrotReference.transforms[0].rotation.x,
            //         _currCarrotReference.transforms[0].rotation.y,
            //         _currCarrotReference.transforms[0].rotation.z,
            //         _currCarrotReference.transforms[0].rotation.w);
            //     break;

            // }
            // else{
            //     _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
            //     _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
            //     _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
            //     _currVisualServoFeed.yaw = util::calculateYaw(
            //         _currOdom.pose.pose.orientation.x,
            //         _currOdom.pose.pose.orientation.y,
            //         _currOdom.pose.pose.orientation.z,
            //         _currOdom.pose.pose.orientation.w);
            //     break;
            // }
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

void carrotReferenceCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint msg){
    _currCarrotReference = msg;
}

void run()
{
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;

    _searchTrajectoryFlag.data = false;
    _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
    ros::spinOnce();
    ros::Duration(1.0).sleep();

	while (ros::ok())
	{  
        // std::cout << _readyForPursuit << std::endl;
		ros::spinOnce();
        if(_readyForPursuit){
            checkDetection();
            updateState();
            _pubSearchTrajectoryFlag.publish(_searchTrajectoryFlag);
            _pubInferenceEnabled.publish(_inferenceEnabled);
            publishOffsets();
            publishErrors();
            publishVisualServoSetpoint(dt);  
            _challengeTime += 1/_rate;
        }
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
    ros::ServiceClient _vsClienCaller, _searchTrajectoryClientCaller, _interceptionTrajectoryClientCaller, _landClient, _estimatorResetCaller;

    /* Offset subscriber and publisher */
    ros::Publisher _pubVssmState, _pubOffsetY, _pubOffsetZ;

    // TakeOff ended
    ros::Subscriber _subReadyForPursuit;
    ros::Publisher _pubInputTrajectory;
    bool _readyForPursuit = false, _gpsPursuitActivated = false, _gpsPursuitFinished = false;
    int _gpsPointIndex = 0, _numOfGPSPoints = 3;
    Global2Local _global2Local;
    trajectory_msgs::MultiDOFJointTrajectory _gpsPoints;
    double _collectTimeout = 5.0, _afterTakeOffHeight, _challengeTime = 0.0, _challengeTimeout = 810.0;
    double _landLat, _landLon, _landHeight;
    bool _hausdorffStatus = false;
    ros::Publisher _pubEstimatorStart;
    bool _hausdorffIn = false, _backupIn = false, _point2In = false;
    bool _interceptionSent = false, _estimatedEverReceived = false;

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
    ros::Subscriber _subHausdorffEstimator, _subCurrentEstimatedTargetPoint, _subEstimatorStatus, _subBackupEstimator;
    float _interceptionZOffset;
    bool _searchEstimated = false;
    geometry_msgs::PointStamped _currentEstimatedTargetPoint;

    /* Pose publisher */
    ros::Publisher _pubVisualServoFeed;
    uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

    /* Odometry subscriber */
    ros::Subscriber _subOdom, _subCarrotReference;
    nav_msgs::Odometry _currOdom;
    trajectory_msgs::MultiDOFJointTrajectoryPoint _currCarrotReference, _lastCarrotReference;

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
    geometry_msgs::PoseStamped _hausdorffPoint, _backupPoint;
    geometry_msgs::PointStamped _point2;


    bool _interceptionActivated = false;
    double _collectTime, _searchTimeOut = 5.0, _followingStartTime = 0, _settleThreshold;
    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _pursuitConfigMutex;
    dynamic_reconfigure::Server<pursuit_param_t>
        _pursuitConfigServer {_pursuitConfigMutex, ros::NodeHandle(PURSUIT_DYN_RECONF)};
    dynamic_reconfigure::Server<pursuit_param_t>::CallbackType _pursuitParamCallback;
};
}

#endif /* VISUAL_SERVO_PURSUIT_STATE_MACHINE_H*/

