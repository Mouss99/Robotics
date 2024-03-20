#include "cubic_polynomial_planner/action.hpp"


ActionServer::ActionServer(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
                                            
															server (node_handle_, "/gen3/action_planner/pose", boost::bind(&ActionServer::action_callback, this, _1 ), false),
															loop_rate_(500)
															{
														//	move_to_action_server_ (node_handle_, "go_to_home_configuration", false) {
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	// prepare all publishers, subscribers, servers, clients, etc
	pose_subscriber_ = node_handle_.subscribe("/gen3/reference/pose", 1, &ActionServer::feedbackCallback, this) ;

    action_counter = 0;

	server.start();

	ROS_INFO_STREAM("[ActionServer::ActionServer] action server is ready");

}

ActionServer::~ActionServer() {

}


void ActionServer::update() {
	
    //calculate distance
    Eigen::VectorXd curr_pos(3);
    Eigen::VectorXd targ_pos(3);

    curr_pos << feedback_pose.position.x, feedback_pose.position.y, feedback_pose.position.z;
    targ_pos <<  target_x, target_y, target_z;

    double distance = (targ_pos - curr_pos).norm();

    
    //calculate the orientation difference
    // Convert the orientation from geometry_msgs/Quaternion to Eigen::Quaternion
    Eigen::Quaterniond q_ini(feedback_pose.orientation.w, feedback_pose.orientation.x, feedback_pose.orientation.y, feedback_pose.orientation.z);

    Eigen::Quaterniond q_tar = Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(target_yaw, Eigen::Vector3d::UnitZ());
    
    // Calculate dot product
    double dot = q_tar.dot(q_ini);

    // Calculate the angle between quaternions
    double theta = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot))); // Ensure dot is within [-1, 1]

    action_feedback.distance_translation = distance;
    action_feedback.distance_orientation = theta;

}

void ActionServer::feedbackCallback(const geometry_msgs::Pose::ConstPtr& msg) {

    feedback_pose = *msg;
	
}


void ActionServer::action_callback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) {

		
    target_x = goal->x;
    target_y = goal->y;
    target_z = goal->z;
    target_roll = goal->roll;
    target_pitch = goal->pitch;
    target_yaw = goal->yaw;
    target_T = goal->T;

    //client for service of kinematic controller
    // subscribe to the service "my_service" with type "hello_msgs::my_service"
	ros::ServiceClient client1 = node_handle_.serviceClient<highlevel_msgs::MoveTo>("/pose_planner/move_to");
    ros::ServiceClient client2 = node_handle_.serviceClient<highlevel_msgs::MoveOri>("/pose_planner/move_ori");

	// make the service MoveTo
	highlevel_msgs::MoveTo srv1;
	srv1.request.x = target_x;
	srv1.request.y = target_y;
    srv1.request.z = target_z;
    srv1.request.T = target_T;

    // make the service MoveOri
	highlevel_msgs::MoveOri srv2;
	srv2.request.x = target_roll;
	srv2.request.y = target_pitch;
    srv2.request.z = target_yaw;
    srv2.request.T = target_T;

    //call the services
    if (client1.call(srv1)) {
        // Handle the response (stored in srv.response)
        ROS_INFO("Translation service call successful!");
    } else {
        ROS_ERROR("Failed to call translation service");
    }

    //call the services
    if (client2.call(srv2)) {
        // Handle the response (stored in srv.response)
        ROS_INFO("Orientation service call successful!");
    } else {
        ROS_ERROR("Failed to call orientation service");
    }


    double time_elapsed = 0.0;

	while (time_elapsed <= target_T) {
        
		server.publishFeedback(action_feedback);

		//ROS_DEBUG_STREAM("[ActionServer::action_callback] distance feedback=" << action_feedback_.distance) ;

        time_elapsed +=0.002;
        
        action_feedback.time_elapsed = time_elapsed;
    
		loop_rate_.sleep() ;
	}
    
    action_result.message = "Action " + std::to_string(action_counter) + " Completed";

    action_counter++;

	server.setSucceeded(action_result) ;

}
