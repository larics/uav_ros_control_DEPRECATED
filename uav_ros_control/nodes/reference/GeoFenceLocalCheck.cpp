#include <ros/ros.h>
#include <uav_ros_control/reference/Global2Local.h>
#include <uav_ros_control/GetLocalConstraints.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include <stdbool.h>


class GeoFenceLocal
{
public: 
	bool getLocalConstraints(
		uav_ros_control::GetLocalConstraints::Request &req,
		uav_ros_control::GetLocalConstraints::Response &resp)
	{
		resp.constraints = _vertices;
		return true;
	}

	GeoFenceLocal(ros::NodeHandle& nh, std::string filename)
	{
		ROS_INFO_STREAM("Loading GPS constraint points from file:\n" << filename);
		YAML::Node config = YAML::LoadFile(filename);
		YAML::Node constraints_list = config["gps_constraints"];
		ROS_INFO_STREAM("Loaded GPS points:\n" << constraints_list);

		// Initialize the conversion class.
		Global2Local global_to_local = Global2Local(nh); 
		ros::Duration(1).sleep();
		ros::spinOnce();

		// Convert GPS constraints to local frame.
		ROS_INFO_STREAM("Constraint points converted to local frame:");
		for (YAML::const_iterator ti = constraints_list.begin(); ti != constraints_list.end(); ++ti)
		{
			const YAML::Node& constraint = *ti;
			double lat = constraint["lat"].as<double>();
			double lon = constraint["lon"].as<double>();
			double alt = constraint["alt"].as<double>();
			Eigen::Vector3d temp_vector = global_to_local.toLocal(lat, lon, alt);
			geometry_msgs::Vector3 vertex;
			vertex.x = temp_vector.x();
			vertex.y = temp_vector.y();
			_vertices.push_back(vertex);
			std::cout << "X: " << vertex.x << ", Y: " << vertex.y << std::endl;
		}
		_vertices.push_back(_vertices[0]);

		// Create a service for returning constraint points in local frame.
	  	_service = nh.advertiseService("get_local_constraints", &GeoFenceLocal::getLocalConstraints, this);
	}
	virtual ~GeoFenceLocal() { }


private:
	std::vector<geometry_msgs::Vector3> _vertices;  // GPS points definining fence polygon
	ros::ServiceServer _service;
};


int main(int argc, char **argv)
{
	// Initialize the node.
	ros::init(argc, argv, "uav_geofence_node");
	ros::NodeHandle nh;

	// Get the name of the file with gps constraints and load them.
	std::string ns = ros::this_node::getNamespace();
	std::string filename;
	nh.getParam(ns + "/gps_constraints_filename", filename);

	GeoFenceLocal gfc = GeoFenceLocal(nh, filename);
	ros::spin();

}
