#pragma once

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>


class KinematicJointController{

	public:
		KinematicJointController(ros::NodeHandle& nodeHandle);
		bool readParameters();
		void call_back(const sensor_msgs::JointState::ConstPtr& msg);
		void update();

		
		
	public:
		//ROS node handle.
		ros::NodeHandle& nh;
		ros::Subscriber subscriber;
		ros::Publisher publisher;

		//current joint state
		std::vector<double> current_joint_pos;

		//to be published
		std_msgs::Float64MultiArray command;


		//parameters
		std::vector<double> t_joint_positions;
		std::vector<double> t_hand_positions;
		std::string urdf_file_name;
		double publish_rate_param;
		double joint_max_velocity;
		double linear_max_velocity;

};
