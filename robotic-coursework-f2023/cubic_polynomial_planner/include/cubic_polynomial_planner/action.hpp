#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <highlevel_msgs/MoveTo.h>
#include <highlevel_msgs/MoveOri.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>



class ActionServer {

	public:
	/*!
	* Constructor.
	* @param node_handle_ the ROS node handle.
	*/
	ActionServer(ros::NodeHandle& node_handle);

  	/*!
  	 * Destructor.
  	 */
	virtual ~ActionServer();

	/*
	 * A function handling necessary actions in every loop
	 */
	void update() ;

	private:
	/*!
	* Reads and verifies the ROS parameters.
	* @return true if successful.
	*/
	//bool readParameters();

    //server_callback
	void action_callback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) ;

    //subscriber call back
	void feedbackCallback(const geometry_msgs::Pose::ConstPtr& msg) ;

	//! ROS node handle.
	ros::NodeHandle& node_handle_;

	//! ROS topic subscriber.
	ros::Subscriber pose_subscriber_ ;

	//! ROS topic name to subscribe to.
	std::string poseSubscriberTopic_ ;

	//! ROS service server.
	ros::ServiceServer serviceServer_;

	//hello_action_type::MoveToGoal action_goal_ ;

	actionlib::SimpleActionServer<highlevel_msgs::PoseCommandAction> server;

	geometry_msgs::Pose feedback_pose;

	highlevel_msgs::PoseCommandGoal action_goal;
	highlevel_msgs::PoseCommandFeedback action_feedback;
    highlevel_msgs::PoseCommandResult action_result;

	double target_x;
    double target_y;
    double target_z;
    double target_roll;
    double target_pitch;
    double target_yaw;
    double target_T;
    
	ros::Rate loop_rate_ 	;
	
	int action_counter;
};


