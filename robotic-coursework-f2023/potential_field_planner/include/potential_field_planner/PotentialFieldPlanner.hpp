#pragma once

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>


class PotentialFieldPlanner{

	public:
		PotentialFieldPlanner(ros::NodeHandle& nodeHandle);
		bool start(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
		void pose_call_back(const geometry_msgs::Pose::ConstPtr &pose_msg);
		void publish_all();
		bool readParameters();
		geometry_msgs::Twist calculateLinearVelocity(geometry_msgs::Pose current_pose, const geometry_msgs::Pose target_pose, double k_att);
		geometry_msgs::Pose calculateDesiredPose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Twist& desired_twist);
		
		
	public:
		//ROS node handle.
		ros::NodeHandle& nh;
		ros::Subscriber subscriber;
		
		//used in service
		ros::Publisher pose_publisher;
		ros::Publisher twist_publisher;
		ros::Publisher done_publisher;

		//desired twist and pose and bool message to publish
		geometry_msgs::Pose desired_pose;
		geometry_msgs::Twist desired_twist;
		geometry_msgs::Pose target_pose;

		//double x;
		double x_;
		double y_;
		double k_att_;
		double linear_max_velocity_;
  		double angular_max_velocity_;
};
