#include "potential_field_planner/PotentialFieldPlanner.hpp"

//for subscriber
geometry_msgs::Pose current_pose;

std_msgs::Bool bool_msg;
bool planning_done;
bool service_called;

double distance_to_target;

//Normalize
double normalize(double a_vel, double max_vel){
        return (a_vel / std::abs(a_vel)) * max_vel;

}

// Function to calculate Euclidean distance
double calculateDistance(double x1, double x2, double y1, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}



PotentialFieldPlanner::PotentialFieldPlanner(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {

     if ( !readParameters() ) {
		    ROS_INFO("Cannot read parameters");
	    }
	    else {
		    ROS_INFO("Successfully read parameters");
	    }

	pose_publisher = nh.advertise<geometry_msgs::Pose>("pose", 1);
    twist_publisher = nh.advertise<geometry_msgs::Twist>("twist", 1);
    done_publisher = nh.advertise<std_msgs::Bool>("done", 1);

	//get current pose and twist
    subscriber = nh.subscribe("/husky_velocity_controller/feedback/pose", 1, &PotentialFieldPlanner::pose_call_back, this);
	
}
	
	
 // HelloParameter::~HelloParameter() {}

bool PotentialFieldPlanner::readParameters() {

    // reading the local parameter
    if (!nh.getParam("/target/x", x_)) {
        ROS_INFO("Cannot read parameter x");
        return false;
    }

    // reading the local parameter
    if (!nh.getParam("/target/y", y_)) {
        ROS_INFO("Cannot read parameter y");
        return false;
    }

    // reading the local parameter
    if (!nh.getParam("/planner/k_att", k_att_)) {
        ROS_INFO("Cannot read parameter k_att");
        return false;
    }

    // reading the local parameter
    if (!nh.getParam("/husky_velocity_controller/linear/x/max_velocity", linear_max_velocity_)) {
        ROS_INFO("Cannot read parameter linear_max_velocity");
        return false;
    }

    // reading the local parameter
    if (!nh.getParam("/husky_velocity_controller/angular/z/max_velocity", angular_max_velocity_)) {
        ROS_INFO("Cannot read parameter angular_max_velocity");
        return false;
    }

    target_pose.position.y = y_;

    ROS_INFO("Parameters:");
    ROS_INFO("x_ : %f", x_);
    ROS_INFO("y_ : %f", y_);
    ROS_INFO("k_att_: %f", k_att_);
    ROS_INFO("linear_max_velocity_ : %f", linear_max_velocity_);

    target_pose.position.x = x_;
    target_pose.position.y = y_;

    ROS_INFO("Parameters:");
    ROS_INFO("x_ : %f", x_);
    ROS_INFO("y_ : %f", y_);
    ROS_INFO("k_att_: %f", k_att_);
    ROS_INFO("linear_max_velocity_ : %f", linear_max_velocity_);
    ROS_INFO("angular_max_velocity_ : %f", angular_max_velocity_);

    return true;
}

//for subscriber
void PotentialFieldPlanner::pose_call_back(const geometry_msgs::Pose::ConstPtr &pose_msg){
	current_pose = *pose_msg;
}

geometry_msgs::Twist PotentialFieldPlanner::calculateLinearVelocity(geometry_msgs::Pose current_pose, const geometry_msgs::Pose target_pose, double k_att) {
    geometry_msgs::Twist vel;
    vel.linear.x += k_att * (target_pose.position.x - current_pose.position.x);

    if (std::abs(vel.linear.x) > linear_max_velocity_){
        normalize(vel.linear.x, linear_max_velocity_);
    }

    vel.linear.y += k_att * (target_pose.position.y - current_pose.position.y);
    if (std::abs(vel.linear.y) > linear_max_velocity_){
        normalize(vel.linear.y, linear_max_velocity_);
    }
    return vel;
}

geometry_msgs::Pose PotentialFieldPlanner::calculateDesiredPose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Twist& desired_twist) {
    geometry_msgs::Pose desired_pose;

    // Calculate the desired position for each dimension individually
    double delta_t = 1.0 / 500.0;
    desired_pose.position.x = current_pose.position.x + desired_twist.linear.x * delta_t;
    desired_pose.position.y = current_pose.position.y + desired_twist.linear.y * delta_t;

    return desired_pose;
}


bool PotentialFieldPlanner::start(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){

    // Return a response to indicate that the service was called successfully
    service_called = true;
    response.success = true;
	return true;

}


void PotentialFieldPlanner::publish_all(){

	planning_done = false;
    //get current pose and twist
    if(service_called) {    

        //distance to reach target
	    distance_to_target = calculateDistance(current_pose.position.x,x_,current_pose.position.y, y_);

    	// Check if the robot has arrived (distance < 0.1)
    	if (distance_to_target < 0.1) {
        planning_done = true;
        service_called = false;
        bool_msg.data = planning_done;
        done_publisher.publish(bool_msg);
    

    	} else {
        planning_done = false;
		desired_twist = calculateLinearVelocity(current_pose,target_pose, k_att_);
        bool_msg.data = planning_done;
		desired_pose= calculateDesiredPose(current_pose, desired_twist);

		pose_publisher.publish(desired_pose);
    	twist_publisher.publish(desired_twist);
        done_publisher.publish(bool_msg);
        
        }
	}

}
