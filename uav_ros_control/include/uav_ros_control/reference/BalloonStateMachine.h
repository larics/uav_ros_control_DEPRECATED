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
#include <uav_ros_control/filters/NonlinearFilters.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <queue>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <uav_ros_control/GiveMeBalloonPosition.h>
#include <geometry_msgs/PointStamped.h>


 namespace uav_reference
{


enum State {
    TAKE_OFF,
    GETNEXTPOINT,
    GOTOPOINT,
    POINTREACHED,
    SPINNING,
    POPPING,
    LAND,
    DONE
};

class BalloonStateMachine
{

public:

BalloonStateMachine(ros::NodeHandle& nh) : _globalToLocal(nh)
{
    // Define Publishers
    _pubState = nh.advertise<std_msgs::Int32>("balloon_sm/state", 1);
    _pubGoal = nh.advertise<geometry_msgs::PoseStamped>("balloon_sm/goal", 1);

    // Define Subscribers
    _subOdom = nh.subscribe("odometry", 1, &BalloonStateMachine::odomCb, this);
    _subPointReached = nh.subscribe("point_reached", 1, &BalloonStateMachine::pointReachedCallback, this);
    _subReadyForExploration = nh.subscribe("ready_for_exploration", 1, &BalloonStateMachine::readyForExplorationCallback, this);

    _carrot_subscriber = nh.subscribe("carrot/trajectory", 1, &BalloonStateMachine::carrotCb, this);

    _landClient = nh.serviceClient<std_srvs::SetBool>("land");
    _giveMeBalloonClient = nh.serviceClient<uav_ros_control::GiveMeBalloonPosition>("/give_me_balloon");

    ros::Duration(1.0).sleep();
    ros::spinOnce();


    // Set initial state
    _currentState = State::TAKE_OFF;

    // publishGeneratedWaypoints();

}

~BalloonStateMachine()
{}

void readyForExplorationCallback(std_msgs::Bool msg){
    _ready = msg.data;
}

void AddLocalWaypoint(double latitude, double longitude, double yaw_requested){
    // Za svaki lat,long dodaj točku za goto + tri točke za spin

    Eigen::Vector3d local = _globalToLocal.toLocal(latitude, longitude, _global_z, true);
    auto odom_q= _currOdom.pose.pose.orientation;

    // calculate yaw
    double yaw = yaw_requested; // - util::calculateYaw(odom_q.x, odom_q.y, odom_q.z, odom_q.w);

    // calculate yaw from curent references and next point

    if (_prva){
        _prva = false;
        _yaw = atan2(local.y() - _currentReference.y, local.x() - _currentReference.x);
    }
    else {
        _yaw = atan2(local.y() - _last_waypoint_y, local.x() - _last_waypoint_x);
    }
    yaw = _yaw;

    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.pose.position.x = local.x();
    tmp_pose.pose.position.y = local.y();
    tmp_pose.pose.position.z = _global_z;

    tf2::Quaternion q;
    q.setRPY(0,0,yaw);
    tmp_pose.pose.orientation.w = q.w();
    tmp_pose.pose.orientation.x = q.x();
    tmp_pose.pose.orientation.y = q.y();
    tmp_pose.pose.orientation.z = q.z();
    _local_waypoints.push(tmp_pose);
    //ROS_WARN("x: %f, y: %f, yaw: %f\n", tmp_pose.pose.position.x, tmp_pose.pose.position.y,
    //         util::calculateYaw(q.x(), q.y(), q.z(), q.w()));

    _last_waypoint_x = local.x();
    _last_waypoint_y = local.y();
}

    void carrotCb(trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory){
    _currentReference = trajectory.transforms[0].translation;
}

void FullRotationArroundZAxis(double x, double y){

    double yaws[] = {0, 3, 6};
    ROS_INFO("Generating spinning\n");

    tf2::Quaternion q;
    for (int i = 0; i < 3; ++i){
        geometry_msgs::PoseStamped tmp_pose = geometry_msgs::PoseStamped();
        tmp_pose.pose.position.x = x;
        tmp_pose.pose.position.y = y;
        tmp_pose.pose.position.z = _global_z;
        q.setRPY(0,0,yaws[i]);
        q.normalize();
        tmp_pose.pose.orientation.w = q.w();
        tmp_pose.pose.orientation.x = q.x();
        tmp_pose.pose.orientation.y = q.y();
        tmp_pose.pose.orientation.z = q.z();
        _spinning_waypoints.push(tmp_pose);
    }

    // Make sure to face next trajectory point

    geometry_msgs::PoseStamped tmp_pose = geometry_msgs::PoseStamped();
    tmp_pose.pose.position.x = x;
    tmp_pose.pose.position.y = y;
    tmp_pose.pose.position.z = _global_z;

    if(!_local_waypoints.empty()) {
        // Get yaw of next waypoint
        auto quat = _local_waypoints.front().pose.orientation;
        tmp_pose.pose.orientation.w = quat.w;
        tmp_pose.pose.orientation.x = quat.x;
        tmp_pose.pose.orientation.y = quat.y;
        tmp_pose.pose.orientation.z = quat.z;

        double yaw_next = util::calculateYaw(quat.x, quat.y, quat.z, quat.w);
        if (abs(yaw_next) < 0.1){
            if (yaw_next > 0) yaw_next = 0.2;
            if (yaw_next <=0) yaw_next = -0.2;
            q.setRPY(0,0,yaw_next);
            tmp_pose.pose.orientation.w = q.w();
            tmp_pose.pose.orientation.x = q.x();
            tmp_pose.pose.orientation.y = q.y();
            tmp_pose.pose.orientation.z = q.z();
            //ROS_WARN("x: %f, y: %f, yaw: %f\n", tmp_pose.pose.position.x, tmp_pose.pose.position.y,
            //         util::calculateYaw(q.x(), q.y(), q.z(), q.w()));
            _spinning_waypoints.push(tmp_pose);
        }
        else {
            _spinning_waypoints.push(tmp_pose);
            //ROS_WARN("x: %f, y: %f, yaw: %f\n", tmp_pose.pose.position.x, tmp_pose.pose.position.y,
            //        util::calculateYaw(quat.x, quat.y, quat.z, quat.w));
        }
    }
}

void pointReachedCallback(const std_msgs::Bool msg) {
    _isPointReached = msg.data;
    ROS_INFO("Current point is reached.");
}

void odomCb(const nav_msgs::Odometry msg){
    _currOdom = msg;
    _curr_yaw = util::calculateYaw(msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);
}

void publishCurrGoal(geometry_msgs::PoseStamped point)
{
    //ROS_INFO("Inside publish current goal!");
    // geometry_msgs::PoseStamped _goalToPub;
    point.header.stamp = ros::Time::now();
    _isPointReached = false;

    ros::Duration(0.2).sleep();
    ROS_WARN("Pub goal");
    _pubGoal.publish(point);
    auto q = point.pose.orientation;
    ROS_WARN("x: %f, y: %f, z: %f yaw: %f\n", point.pose.position.x, point.pose.position.y, point.pose.position.z,
            util::calculateYaw(q.x, q.y, q.z, q.w));
}

double distance(geometry_msgs::Point point, Eigen::Vector3d vector){
    return sqrt(
            (point.x - vector.x()) * (point.x - vector.x())+
            (point.y - vector.y()) * (point.y - vector.y())+
            (point.z - vector.z()) * (point.z - vector.z())
            );
}

bool should_i_spin(){
    //_helper_position_1 = _globalToLocal.toLocal(_lat_, _long_home, _global_z, true);
    _helper_position_2 = _globalToLocal.toLocal(_lat_start_help, _long_start_help, _global_z, true);
    _helper_position_3 = _globalToLocal.toLocal(_lat_end2, _long_end2, _global_z, true);

    return !(//distance(_tmp_position, _helper_position_1) < 1 ||
             distance(_tmp_position, _helper_position_2) < 1 ||
             distance(_tmp_position, _helper_position_3) < 1);
}

void publish_spinning_points(){

    while (!_spinning_waypoints.empty()){
        ros::spinOnce();

        if (_isPointReached) {
            publishCurrGoal(_spinning_waypoints.front());
            ROS_INFO("Spinning in place");
            _isPointReached = false;
            _spinning_waypoints.pop();
        }
    }
}

void destroy(geometry_msgs::PointStamped target_point){

    ROS_INFO("Calling destroy!");

    geometry_msgs::PoseStamped target_pose;
    // Odi metar iznad
    double z_offset = 0.0;
    target_pose.pose.position.x = target_point.point.x;
    target_pose.pose.position.y = target_point.point.y;
    target_pose.pose.position.z = target_point.point.z + z_offset;

    // Todo orientation

    publishCurrGoal(target_pose);
    _isPointReached = false;
    ROS_INFO("Moving towards target");

    while(!_isPointReached){
        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }
    // point reached
    ROS_INFO("Lingering around the target");
    ros::Duration(_sleep_duration).sleep();

    // Vrati se na saved_x, saved_z, global_z, (yaw prema _local_waypoint.front(), ako postoji)
    target_pose.pose.position.x = _saved_x;
    target_pose.pose.position.y = _saved_y;
    target_pose.pose.position.z = _global_z;
    target_pose.pose.orientation = _tmp_orientation;
    publishCurrGoal(target_pose);
    ROS_INFO("Returning to trajectory point");
    _isPointReached = false;
    while(!_isPointReached){
        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }
    // Point reached
    ros::Duration(_sleep_duration).sleep();

}

void move_to_land(){
    ROS_INFO("Moving to landing area");
    _local_land_coords = _globalToLocal.toLocal(_lat_land, _long_land, _global_z, true);
    _tmp_pose = geometry_msgs::PoseStamped();
    _tmp_pose.pose.position.x = _local_land_coords.x();
    _tmp_pose.pose.position.y = _local_land_coords.y();
    _tmp_pose.pose.position.z = _local_land_coords.z();

    publishCurrGoal(_tmp_pose);
    _isPointReached = false;
}

    void land_or_redo(){
        if (!_was_detected) {
            runs_without_detection++;
    }
    if (runs_without_detection == runs_without_detection_param) {
        move_to_land();
        ROS_INFO("[Balloon_sm] Transition to LAND");
        _currentState = State::LAND;
    } else {
        ROS_INFO("Redo trajectory");
        generate_local_waypoints();
        _currentState = State::GETNEXTPOINT;
    }
    ROS_INFO("Runs without detection: %d\n", runs_without_detection);
}

void generate_local_waypoints(){
    for (auto glob_waypoint : _global_waypoints){
        AddLocalWaypoint(glob_waypoint.first, glob_waypoint.second, 0);
    }
}
/******************************************************************************************************************
 * **********************************************State machine****************************************************
 *
 */
void stateAction(){
    switch(_currentState){
        case State::TAKE_OFF:

            _was_detected = false;
            if (_ready){
                ROS_INFO("Ready for action\n");
                _global_z = _currentReference.z;
                generate_local_waypoints();
                ROS_INFO("[Balloon_sm] Transition to GETNEXTPOINT");
                _currentState = State::GETNEXTPOINT;
            }

            break;
        case State::GETNEXTPOINT:

            ROS_WARN("[Balloon_sm] Get next point\n Points left: %d", _local_waypoints.size());

                publishCurrGoal(_local_waypoints.front());
                _tmp_position = _local_waypoints.front().pose.position;
                _tmp_orientation = _local_waypoints.front().pose.orientation;
                ROS_INFO("[Balloon_sm] Transition to GOTOPOINT - moving to next waypoint");
                _currentState = State::GOTOPOINT;

            break;

        case State::GOTOPOINT:


            if (_isPointReached){
                ROS_INFO("[Balloon_sm] Transition to POINTREACHED");
                _currentState = State::POINTREACHED;
            }

           break;

        case State::POINTREACHED:

            ros::Duration(_sleep_duration).sleep();
            _saved_x = _tmp_position.x;
            _saved_y = _tmp_position.y;
            _local_waypoints.pop();
            if (should_i_spin()){
                ROS_INFO("We should spinn\n x: %f \n y:%f \n", _saved_x, _saved_y);
                _currentState = State::SPINNING;
                FullRotationArroundZAxis(_saved_x, _saved_y);
                break;
            }
            else{
                if (!_local_waypoints.empty()) {
                    ROS_INFO("[Balloon_sm] Transition to GETNEXTPOINT");
                    _currentState = State::GETNEXTPOINT;
                }
                else {
                    land_or_redo();
                }
            }

            break;

        case State::SPINNING:

            publish_spinning_points();

            if (_isPointReached) {
                ROS_INFO("[Balloon_sm] Transition to POPPING");
                _currentState = State::POPPING;
            }

            break;
        case State::POPPING :
            ROS_WARN("[Balloon_sm] Popping\n");

            // Check for balloons

            _giveMeBalloonClient.call(_giveMeBalloonRequest, _giveMeBalloonResponse);
            if(_giveMeBalloonResponse.status){
                while(_giveMeBalloonResponse.status) {
                    ROS_INFO("Balloon found");
                    // Todo spin i delay su u destroy
                    destroy(_giveMeBalloonResponse.balloon_position);
                    _giveMeBalloonClient.call(_giveMeBalloonRequest, _giveMeBalloonResponse);
                    _was_detected = true;
                    runs_without_detection = 0;
                }
            }

            if(_local_waypoints.empty()){
                land_or_redo();
            }
            else {
                ROS_INFO("[Balloon_sm] Transition to GETNEXTPOINT");
                _currentState = State::GETNEXTPOINT;
            }
            break;


        case State::LAND:

            if(_isPointReached){
                ROS_INFO("calling land.");
                land_srv.data = true;
                _landClient.call(land_srv, service_success);
                if (service_success.success) {
                    ROS_INFO("Landed.");
                    ROS_WARN("[Balloon_sm] Land\n");
                    _currentState = State::DONE;
                }
                else{
                    ROS_ERROR("[Balloon SM] Calling land failed.\n Message: %s",service_success.message.data());
                    _currentState = State::LAND;
                }
            }


            break;
    }


}


void run()
{
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;
	while (ros::ok())
	{
		ros::spinOnce();
		stateAction();
		_intMsg.data = _currentState;
		_pubState.publish(_intMsg);
        loopRate.sleep();
    }
}

private:

    std_srvs::SetBoolRequest land_srv;
    std_srvs::SetBoolResponse service_success;

    ros::ServiceClient _landClient, _giveMeBalloonClient;
    uav_ros_control::GiveMeBalloonPositionRequest  _giveMeBalloonRequest;
    uav_ros_control::GiveMeBalloonPositionResponse _giveMeBalloonResponse;


    double _rate = 50;

    /* Service Pursuit */
	ros::ServiceServer _servicePursuit;
    bool _pursuitActivated = false;
    bool _isPointReached = false;
    bool _prva = true;
    bool _spinning = false;
    bool _new_spinning = false;
    bool _ready = false;
    bool _takeoff_success = false;

    double _temp_yaw;
    double _curr_yaw;
    double _currentReferenceYaw;

    double _last_waypoint_x, _last_waypoint_y;

    State _currentState;

    ros::Publisher _pubGoal, _pubState;

    std_msgs::Int32 _intMsg;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;

    ros::Subscriber _subPointReached;

    nav_msgs::Odometry _currOdom;

    ros::Subscriber _carrot_subscriber;

    // Todo: set as param

    // sim
    // double _lat_home = -35.3632631;
    // double _long_home = 149.165237;

    // const double _lat_start_help = -35.3632631, _long_start_help=149.165247;

    // const double _lat_start = -35.3632631, _long_start =149.165287;
    // const double _lat_mid_1 = -35.3632631, _long_mid_1 =149.165327;
    // const double _lat_mid_2 = -35.3632631, _long_mid_2 =149.165367;
    // const double _lat_end = -35.3632631, _long_end =149.165407;
    // const double _lat_end2 = -35.3632631, _long_end2 =149.165387;
    // const double _lat_land = -35.3632631, _long_land=149.165247;


    // vani
    const double _lat_start_help = 24.41776, _long_start_help = 54.43562;
    const double      _lat_start = 24.41775,      _long_start = 54.43565;
    const double      _lat_mid_1 = 24.4177,      _long_mid_1 = 54.43583;
    const double      _lat_mid_2 = 24.41764,      _long_mid_2 = 54.43608;
    const double        _lat_end = 24.41760,        _long_end = 54.43626;
    const double       _lat_end2 = 24.41759,       _long_end2 = 54.43629;
    const double       _lat_land = _lat_start_help,_long_land = _long_start_help;

    // vanjske u sim
    //double _lat_sim_home = -35.3632631;
    // double _long_sim_home = 149.165237;
    // double _lat_offset = 24.41775, _long_offset = 54.43559;
    // const double _lat_start_help = 24.41775 - _lat_offset + _lat_sim_home,  _long_start_help = 54.43559 - _long_offset + _long_sim_home;
    // const double      _lat_start = 24.41772 - _lat_offset  + _lat_sim_home,   _long_start = 54.43562 - _long_offset + _long_sim_home;
    // const double      _lat_mid_1 = 24.41767 - _lat_offset + _lat_sim_home,   _long_mid_1 = 54.43582 - _long_offset + _long_sim_home;
    // const double      _lat_mid_2 = 24.41760 - _lat_offset + _lat_sim_home,   _long_mid_2 = 54.43610 - _long_offset + _long_sim_home;
    // const double        _lat_end = 24.41756 - _lat_offset + _lat_sim_home,     _long_end = 54.43626 - _long_offset + _long_sim_home;
    // const double       _lat_end2 = 24.41757 - _lat_offset + _lat_sim_home,    _long_end2 = 54.43625 - _long_offset + _long_sim_home;
    // const double       _lat_land = 24.4176690 - _lat_offset + _lat_sim_home,   _long_land = 54.4359773 - _long_offset + _long_sim_home;

    double _global_z, _yaw, _saved_x, _saved_y;
    geometry_msgs::Quaternion _tmp_orientation;

    geometry_msgs::Vector3 _currentReference;


    std::vector<std::pair<double, double>> _global_waypoints{
            {_lat_start_help, _long_start_help},
            {_lat_start,_long_start},
            {_lat_mid_1,_long_mid_1},
            {_lat_mid_2,_long_mid_2},
            {_lat_end,  _long_end},
            {_lat_end2, _long_end2},
            {_lat_mid_2,_long_mid_2},
            {_lat_mid_1,_long_mid_1},
            {_lat_start,_long_start}
    };

    std::queue<geometry_msgs::PoseStamped> _local_waypoints;
    std::queue<geometry_msgs::PoseStamped> _spinning_waypoints;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _pursuitConfigMutex;

    Global2Local _globalToLocal;
    ros::Subscriber _subReadyForExploration;

    geometry_msgs::Point _tmp_position;
    geometry_msgs::PoseStamped _tmp_pose;
    geometry_msgs::PoseStamped _tmp_spinning_pose;
    double _tmp_spinning_yaw;
    double _sleep_duration = 2;
    Eigen::Vector3d _helper_position_1, _helper_position_2, _helper_position_3, _local_land_coords;
    int runs_without_detection = 0;
    int runs_without_detection_param = 1;
    bool _was_detected = false;
};
}

#endif /* VISUAL_SERVO_PURSUIT_STATE_MACHINE_H*/
