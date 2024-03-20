#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/math/quaternion.hpp>

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <highlevel_msgs/MoveTo.h>
#include <highlevel_msgs/MoveOri.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


class KinematicTaskController{

	public:
		KinematicTaskController(ros::NodeHandle& nodeHandle);
		bool readParameters();
		void subscriber_call_back(const sensor_msgs::JointState::ConstPtr& msg);
		bool move_service_call_back (highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res);
		bool interpolation_service_call_back (highlevel_msgs::MoveOri::Request &req, highlevel_msgs::MoveOri::Response &res);
		void update();
		void forward_kinematics();
		Eigen::MatrixXd computeJacobian();
		
		
	public:
		//ROS node handle.
		ros::NodeHandle& nh;
		ros::Subscriber subscriber;
		ros::Publisher publisher;
		ros::Publisher pose_publisher;
		ros::Publisher twist_publisher;
		bool ready;
		//ros::ServiceServer service;

		//current joint state
		std::vector<double> current_joint_pos;
		std::vector<double> current_joint_vel;
		Eigen::VectorXd curr_pos;
		// Eigen::MatrixXd  current_hand_ori;
		Eigen::VectorXd x_vel_fbk;
		Eigen::VectorXd x_pos_ref;
		Eigen::VectorXd x_vel_ref;

		Eigen::Vector3d targ_pos;
    	Eigen::Vector3d targ_ori;

		//to be published
		std_msgs::Float64MultiArray command;

		//service call bool
		bool service_called;
		bool ori_service_called;

		
		//start time for cubic polynomial
		double start_time;
		double ori_start_time;
		bool default_pos;
		bool default_ori;

		//move service input
		double target_x;
		double target_y;
		double target_z;
		double target_T;

		//move service input
		double target_roll;
		double target_pitch;
		double target_yaw;
		double target_T_ori;

		//parameters
		std::vector<double> t_joint_positions;
		std::vector<double> t_hand_positions;
		std::vector<double> t_hand_ori;
		std::string urdf_file_name;
		double publish_rate_param;
		double joint_max_velocity;
		double linear_max_velocity;
		double angular_max_velocity;

		//for pinocchio
		pinocchio::Model model_;
		pinocchio::Data data_ ;
};	
