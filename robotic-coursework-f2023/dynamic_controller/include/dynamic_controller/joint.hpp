#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>


class DynamicJointController{

	public:
		DynamicJointController(ros::NodeHandle& nodeHandle);
		bool readParameters();
		void call_back(const sensor_msgs::JointState::ConstPtr& msg);
		std::vector<Eigen::VectorXd> plan();
		void update();
		bool ready();

		
		
	public:
		//ROS node handle.
		ros::NodeHandle& nh;
		ros::Subscriber subscriber;
		ros::Publisher publisher;

		//current joint state
		std::vector<double> current_joint_pos;
		std::vector<double> current_joint_vel;

		//to be published
		std_msgs::Float64MultiArray command;


		//parameters
		std::vector<double> t_joint_positions;
		std::vector<double> t_hand_positions;
		std::string urdf_file_name;
		double publish_rate_param;
		double joint_max_velocity;
		double linear_max_velocity;

		//for pinocchio
		pinocchio::Model model_;
		pinocchio::Data data_ ;
};
