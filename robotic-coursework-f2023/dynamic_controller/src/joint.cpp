#include "dynamic_controller/joint.hpp"

DynamicJointController::DynamicJointController(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {

    if ( !readParameters() ) {
		ROS_INFO("Cannot read parameters");
	}
	else {
		ROS_INFO("Successfully read parameters");
	}
    
    pinocchio::urdf::buildModel(urdf_file_name, model_, false);
	data_ = pinocchio::Data(model_);

    //subscriber = nh.subscribe("/gen3/joint_states", 1, &TaskSpacePlanner::subscriber_call_back, this);
    publisher = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command", 1);

    current_joint_pos.resize(7,0.0);
    current_joint_vel.resize(7,0.0);

    subscriber = nh.subscribe("/gen3/joint_states", 1, &DynamicJointController::call_back, this);


}


bool DynamicJointController::readParameters() {
    
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

void DynamicJointController::call_back(const sensor_msgs::JointState::ConstPtr& msg){

    current_joint_pos = msg -> position;
    current_joint_vel = msg -> velocity;

}


std::vector<Eigen::VectorXd> DynamicJointController::plan(){

        //joint position, velocity and acceleration
        Eigen::VectorXd q_vel;
        Eigen::VectorXd q_pos;
        Eigen::VectorXd q_acc;

        q_vel.resize(7);
        q_pos.resize(7);
        q_acc.resize(7);
        
        //calculate velocities
        for (int i = 0; i < 7; i++){

            //current position of a joint
            double qc = current_joint_pos[i];
            
            double qf = t_joint_positions[i];
            //set k_att to 8    
            q_vel[i] = 8.0*(qf - qc);
            
        }

        //normalize joint velocity
        if (q_vel.norm() > joint_max_velocity){
                q_vel = joint_max_velocity * (q_vel / q_vel.norm());
        }


        for (int i = 0; i < 7; i++){

            double qc = current_joint_pos[i];

            //get position
            q_pos[i] = (qc + 60*q_vel[i]*(1.0/publish_rate_param));

            //get acceleration
            q_acc[i] = (q_vel[i]-current_joint_vel[i])/(publish_rate_param);
        }   


        //return array of eigen vectors
        std::vector<Eigen::VectorXd> potential_functions;
        potential_functions.push_back(q_pos);
        potential_functions.push_back(q_vel);
        potential_functions.push_back(q_acc);

        return potential_functions;

}


void DynamicJointController::update(){

        if (ready()){
        
    
        //compute all pinnochio terms
        Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());
        Eigen::VectorXd joint_vel = Eigen::Map<Eigen::VectorXd>(current_joint_vel.data(), current_joint_vel.size());
        pinocchio::computeAllTerms(model_, data_, joint_pos, joint_vel);

        std::vector<Eigen::VectorXd> potential_functions = plan();

        
        //use pinocchio to get M and h
        //7x1
        Eigen::VectorXd h_(data_.nle.rows(), data_.nle.cols());
        //7x7
        Eigen::MatrixXd M_(data_.M.rows(), data_.M.cols());

        h_ = data_.nle;
        M_ = data_.M;


        //joint velocity vector
        Eigen::VectorXd q_vel;
        Eigen::VectorXd q_pos;
        Eigen::VectorXd q_acc;
        q_vel.resize(7);
        q_pos.resize(7);
        q_acc.resize(7);

        q_pos = potential_functions[0];
        q_vel = potential_functions[1];
        q_acc = potential_functions[2];

        Eigen::VectorXd q_double_dot;
        q_double_dot.resize(7);

        //calculate q double dot command
        for (int i = 0; i < 7; i++){

            int K_ = 80;
            int D_ = 5;

            q_double_dot[i] = q_acc[i] + D_*(q_vel[i] - current_joint_vel[i]) + K_*(q_pos[i] - current_joint_pos[i]);

        }

        //Calculate torque
        Eigen::VectorXd torque = M_*q_double_dot + h_;


        command.data.clear();
        //fill up array to publish command
        for (int i = 0; i < 7; i++){

            command.data.push_back(torque[i]);
        }

        publisher.publish(command);
        }
}

bool DynamicJointController::ready(){

    Eigen::VectorXd feedback = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());

    return (feedback.norm() > 0);

}

