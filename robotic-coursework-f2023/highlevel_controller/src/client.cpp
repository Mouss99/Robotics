#include "highlevel_controller/client.hpp"


ActionClient::ActionClient(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															action_client_ ("/gen3/action_planner/pose", true) {

	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}


	// prepare all publishers, subscribers, servers, clients, etc
    //feedback_subscriber_ 	= node_handle_.subscribe(feedback_topic_name_, 1, 	&ActionClient::feedbackCallback, this);

	ROS_INFO_STREAM("Successfully launched node.") ;
}

ActionClient::~ActionClient() {

}

bool ActionClient::readParameters() {
	// reading the local parameter
    if (!node_handle_.getParam("/action_list/number_of_targets", NUM_TARGETS_)) {
        ROS_INFO("Cannot read parameter for number of targets");
        return false;
    }

    action_data_	= Eigen::MatrixXd::Random(NUM_TARGETS_,7) ; // instead of random numbers, read it from YAML

    for (int i = 0; i < NUM_TARGETS_; ++i) {

        std::string action_param_prefix = "action_list/action_" + std::to_string(i);
        Eigen::VectorXd translation(3);
        Eigen::VectorXd orientation(3);

        std::vector<double> tempVector1;
        std::vector<double> tempVector2;

        if (!node_handle_.getParam(action_param_prefix + "/translation", tempVector1)) {
            ROS_INFO("Failed to get the translation parameter");
            return false;
        }

        if (!node_handle_.getParam(action_param_prefix + "/orientation", tempVector2)) {
            ROS_INFO("Failed to get the orientation parameter");
            return false;
        }

        double duration = 0.0;
        if (!node_handle_.getParam(action_param_prefix + "/duration", duration)) {
            ROS_INFO("Failed to get the duration parameter");
            return false;
        }

        //convert from std vector to eigen
        translation = Eigen::Map<Eigen::VectorXd>(tempVector1.data(), tempVector1.size());
        orientation = Eigen::Map<Eigen::VectorXd>(tempVector2.data(), tempVector2.size());

        // Set the data for the current action in the action_data matrix
        action_data_(i, 0) = translation(0);
        action_data_(i, 1) = translation(1);
        action_data_(i, 2) = translation(2);
        action_data_(i, 3) = orientation(0);
        action_data_(i, 4) = orientation(1);
        action_data_(i, 5) = orientation(2);
        action_data_(i, 6) = duration;

        //ros::Duration(duration).sleep();
    }
    
    // Print the gathered data for demonstration
    for (int i = 0; i < NUM_TARGETS_; ++i) {
        ROS_INFO_STREAM("Action " << i << " Data: ["
                        << action_data_(i, 0) << ", " << action_data_(i, 1) << ", " << action_data_(i, 2) << ", "
                        << action_data_(i, 3) << ", " << action_data_(i, 4) << ", " << action_data_(i, 5) << ", "
                        << action_data_(i, 6) << "]");
    }

	return true;
}

void ActionClient::update() {

	ROS_INFO_STREAM("[ActionClient::update] Action client is ready. Wait for the server...") ;

	//boost::function fun_done = boost::bind(&ActionClient::doneCallback, this, _1, _2) ;

	for ( int counter = 0 ; counter < NUM_TARGETS_ ; counter++ ) {

		// Fill in goal here
		action_goal_.x = action_data_(counter, 0) ;
		action_goal_.y = action_data_(counter, 1)  ;
		action_goal_.z = action_data_(counter, 2)  ;
        action_goal_.roll = action_data_(counter, 3) ; 
        action_goal_.pitch = action_data_(counter, 4) ; 
        action_goal_.yaw = action_data_(counter, 5) ; 
        action_goal_.T = action_data_(counter, 6) ; 

		// wait for the server
		action_client_.waitForServer();

		action_client_.sendGoal(action_goal_,
				boost::bind(&ActionClient::doneCallback, this, _1, _2),
				boost::bind(&ActionClient::activeCallback, this),
				boost::bind(&ActionClient::feedbackCallback, this, _1) ) ;

		ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
		action_client_.waitForResult( ros::Duration(30.0) );

		if ( action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
		}
	}


	ROS_INFO_STREAM("[ActionClient::doneCallback] Done, shotdown") ;
	ros::shutdown();
}


void ActionClient::doneCallback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) {
	ROS_INFO("[ActionClient::doneCallback] Finished in state [%s]", state.toString().c_str());
}

void ActionClient::activeCallback() {
	ROS_INFO_STREAM("[ActionClient::activeCallback] Action has become active");
}


void ActionClient::feedbackCallback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) {
	ROS_DEBUG_STREAM("[ActionClient::feedbackCallback]" << feedback->time_elapsed) ;
}

