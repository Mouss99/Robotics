#include "kinematic_controller/joint.hpp"

KinematicJointController::KinematicJointController(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {

    if ( !readParameters() ) {
		ROS_INFO("Cannot read parameters");
	}
	else {
		ROS_INFO("Successfully read parameters");
	}
    
    subscriber = nh.subscribe("/gen3/joint_states", 1, &KinematicJointController::call_back, this);
    publisher = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 1);
    current_joint_pos.resize(7,0.0);

}


bool KinematicJointController::readParameters() {
    
// reading the local parameter
    if (!nh.getParam("/gen3/target/joint/positions", t_joint_positions)) {
        ROS_INFO("Cannot read parameter for target joint positions");
        return false;
    }
    
    // reading the local parameter
    if (!nh.getParam("/gen3/target/hand/position", t_hand_positions)) {
        ROS_INFO("Cannot read parameter for target hand positions");
        return false;
    }
    // reading the local parameter
    if (!nh.getParam("/gen3/urdf_file_name", urdf_file_name)) {
        ROS_INFO("Cannot read parameter for udrf file name");
        return false;
    }

    // reading the local parameter
    if (!nh.getParam("/publish_rate", publish_rate_param)) {
        ROS_INFO("Cannot read parameter for publish rate");
        return false;
    }


    // reading joint max velocity param
    if (!nh.getParam("/gen3/joint/max_velocity", joint_max_velocity)) {
        ROS_INFO("Cannot read parameter for max joint velocity");
        return false;
    }

    // reading joint max velocity param
    if (!nh.getParam("/gen3/linear/max_velocity", linear_max_velocity)) {
        ROS_INFO("Cannot read parameter for max linear velocity");
        return false;
    }

    for (int i = 0; i < t_joint_positions.size(); ++i){
        ROS_INFO("joint position [%d]: %f", i, t_joint_positions[i]);
    }

    for (int i = 0; i < t_hand_positions.size(); ++i){
        ROS_INFO("hand position [%d]: %f", i, t_hand_positions[i]);
    }

    ROS_INFO("urdf file name : %s", urdf_file_name.c_str()); 
    ROS_INFO("Publish rate : %f", publish_rate_param);
    ROS_INFO("Joint max velocity : %f", joint_max_velocity);
    ROS_INFO("linear max velocity : %f", linear_max_velocity);

    return true;


}

void KinematicJointController::call_back(const sensor_msgs::JointState::ConstPtr& msg){

    current_joint_pos = msg -> position;

}


void KinematicJointController::update(){

        Eigen::VectorXd curr_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());
        Eigen::VectorXd targ_pos = Eigen::Map<Eigen::VectorXd>(t_joint_positions.data(), t_joint_positions.size());

        //joint velocity vector
        Eigen::VectorXd q_vel;
        q_vel.resize(7);
        q_vel = 8.0*(targ_pos - curr_pos);

        //normalize joint velocity
        if (q_vel.norm() > joint_max_velocity){
                q_vel = joint_max_velocity * (q_vel / q_vel.norm());
        }

        //calculate desired joint position
        Eigen::VectorXd q_ref;
        q_ref.resize(7);
        q_ref = curr_pos + (5*q_vel)/publish_rate_param;
        
        command.data.clear();
        for (int i = 0; i < 7; i++){
            command.data.push_back (q_ref[i]);
        }
        

        publisher.publish(command);

}
