#include "kinematic_controller/task.hpp"


KinematicTaskController::KinematicTaskController(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {

    if ( !readParameters() ) {
		ROS_INFO("Cannot read parameters");
	}
	else {
		ROS_INFO("Successfully read parameters");
	}

    pinocchio::urdf::buildModel(urdf_file_name, model_, false);
	data_ = pinocchio::Data(model_);

    //subscriber = nh.subscribe("/gen3/joint_states", 1, &TaskSpacePlanner::subscriber_call_back, this);
    publisher = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 1);

    pose_publisher = nh.advertise<geometry_msgs::Pose>("/gen3/reference/pose", 1);
    twist_publisher = nh.advertise<geometry_msgs::Twist>("/gen3/reference/twist", 1);

    subscriber = nh.subscribe("/gen3/joint_states", 1, &KinematicTaskController::subscriber_call_back, this);

    current_joint_pos.resize(7);
    current_joint_vel.resize(7);

    curr_pos.resize(6);
    x_pos_ref.resize(3);
    x_vel_ref.resize(3);

    targ_ori.resize(3);
    targ_pos.resize(3);
    
    default_pos = true;
    default_ori = true;
    service_called = false;
    ori_service_called = false;
    ready=false;


    targ_pos = Eigen::Map<Eigen::VectorXd>(t_hand_positions.data(), t_hand_positions.size());
    targ_ori = Eigen::Map<Eigen::VectorXd>(t_hand_ori.data(), t_hand_ori.size());
}


bool KinematicTaskController::readParameters() {
    
  
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
    if (!nh.getParam("/gen3/target/hand/orientation", t_hand_ori)) {
        ROS_INFO("Cannot read parameter for target hand orientation");
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

    // reading joint max velocity param
    if (!nh.getParam("/gen3/angular/max_velocity", angular_max_velocity)) {
        ROS_INFO("Cannot read parameter for max angular velocity");
        return false;
    }

    for (int i = 0; i < t_joint_positions.size(); ++i){
        ROS_INFO("joint position [%d]: %f", i, t_joint_positions[i]);
    }

    for (int i = 0; i < t_hand_positions.size(); ++i){
        ROS_INFO("hand position [%d]: %f", i, t_hand_positions[i]);
    }

    for (int i = 0; i < t_hand_ori.size(); ++i){
        ROS_INFO("hand orientation [%d]: %f", i, t_hand_ori[i]);
    }

    ROS_INFO("urdf file name : %s", urdf_file_name.c_str()); 
    ROS_INFO("Publish rate : %f", publish_rate_param);
    ROS_INFO("Joint max velocity : %f", joint_max_velocity);
    ROS_INFO("linear max velocity : %f", linear_max_velocity);
    ROS_INFO("angular max velocity : %f", angular_max_velocity);

    return true;

}

void KinematicTaskController::subscriber_call_back(const sensor_msgs::JointState::ConstPtr& msg){

    current_joint_pos = msg -> position;
    current_joint_vel = msg -> velocity;

    ready = true;

}

bool KinematicTaskController::move_service_call_back (highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res) {

    // Read input   
    target_x = req.x;
    target_y = req.y;
    target_z = req.z;
    target_T = req.T;

    //input verification
    if (target_z <= 0 || target_T <= 0) {
        ROS_ERROR("Target position z=0 is not allowed. Robot may crash.");
        res.success = false;
        return false;
    }
    
    //if input is correct, annouce service start and return true
    service_called = true;
    res.success = true;
    default_pos = false;
    start_time = ros::Time::now().toSec();
    return true;

}

bool KinematicTaskController::interpolation_service_call_back (highlevel_msgs::MoveOri::Request &req, highlevel_msgs::MoveOri::Response &res) {

    // Read input   
    target_roll = req.x;
    target_pitch = req.y;
    target_yaw = req.z;
    target_T_ori = req.T;

    //input verification
    if (target_T <= 0) {
        ROS_ERROR("Target time is not allowed");
        res.success = false;
        return false;
    }
    
    //if input is correct, annouce service start and return true
    ori_service_called = true;
    res.success = true;
    default_ori = false;
    ori_start_time = ros::Time::now().toSec();
    return true;

}



void KinematicTaskController::forward_kinematics(){ 
    
    const int JOINT_ID = 7;
    int dim_joints = model_.nq ;

    Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());
	

	// TEST FORWARD KINEMATICS
	pinocchio::forwardKinematics(model_, data_, joint_pos);							// forward kinematics
	pinocchio::SE3 pose_now = data_.oMi[JOINT_ID];												// end-effector pose

    // Extract the end-effector position (translation components)
    Eigen::Vector3d translation = pose_now.translation();
    Eigen::Vector3d orientation = pinocchio::rpy::matrixToRpy(pose_now.rotation());

    curr_pos.head<3>() = translation;
    curr_pos.tail<3>() = orientation;


}

Eigen::MatrixXd KinematicTaskController::computeJacobian(){
    const int JOINT_ID = 7;
    int dim_joints = model_.nq ;

    Eigen::MatrixXd jacobian_ = Eigen::MatrixXd::Zero(6,dim_joints) ;
    
    //get current joint pos and map it to eigen 
    Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());

    Eigen::VectorXd joint_vel = Eigen::Map<Eigen::VectorXd>(current_joint_vel.data(), current_joint_vel.size());

    pinocchio::computeAllTerms(model_, data_, joint_pos, joint_vel) ;

    // Ensure you have model and joint_pos properly defined.
    pinocchio::getJointJacobian(model_, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);

    return jacobian_;
}


void KinematicTaskController::update(){
    
    forward_kinematics();
    double k_att = 8.0;

    //if service call, use cubic polynomial
    if (service_called){

            targ_pos << target_x, target_y, target_z; 
            service_called = false;
    }

    //if trying to send arm to default position, use the potential field
    if (default_pos){

        targ_pos = Eigen::Map<Eigen::VectorXd>(t_hand_positions.data(), t_hand_positions.size());
        default_pos = true;
    }

    if (ori_service_called){
            targ_ori << target_roll, target_pitch, target_yaw;
            ori_service_called = false;
    }
        

    if(default_ori){
        targ_ori = Eigen::Map<Eigen::VectorXd>(t_hand_ori.data(), t_hand_ori.size());
        default_ori = true;
    }

    x_vel_ref = k_att * (targ_pos - curr_pos.head<3>());

    //normalize linear vel
    if (x_vel_ref.norm() > linear_max_velocity){
        x_vel_ref = linear_max_velocity * (x_vel_ref / x_vel_ref.norm());
    }
    x_pos_ref = curr_pos.head<3>() + 10*(x_vel_ref/publish_rate_param);

    pinocchio::SE3 pose_now = data_.oMi[7];
            
    Eigen::MatrixXd rot_targ_ori = pinocchio::rpy::rpyToMatrix(targ_ori[0],targ_ori[1], targ_ori[2]);

    pinocchio::SE3 pose_ref;
            
    pose_ref.translation() = targ_pos;
    pose_ref.rotation() = rot_targ_ori;

    //the error between two homegeneous transformations
    pinocchio::SE3 pose_err ;
    pose_err = pose_now.inverse() * pose_ref;

    // pose error vector in local frame (6x1 )
    Eigen::VectorXd pose_err_vec_local;
    pose_err_vec_local.resize(6);

    // pose error vector in global frame (6x1 )
    Eigen::VectorXd pose_err_vec_world;
    pose_err_vec_world.resize(6);

    // convert it to Euler angles in local frame
    pose_err_vec_local = pinocchio::log6 (pose_err).toVector();

    // convert the euler angles from local frame to world frame (3x1 )
    pose_err_vec_world.tail<3>() = (data_.oMi[7].rotation())*pose_err_vec_local.tail<3>();

    //compute jacobian
    Eigen::MatrixXd jacobian = computeJacobian();

    //compute pseudo inverse
    Eigen::MatrixXd pseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    //identity matrix and nullspace for redundancy
    Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd nullspace = identity_matrix - (pseudoInverse*jacobian);

    //q zero for redundancy
    Eigen::VectorXd q_zero_pos = Eigen::Map<Eigen::VectorXd>(t_joint_positions.data(), t_joint_positions.size());
    Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());

    //find home q velocity
    Eigen::VectorXd q_dot_zero = k_att * (q_zero_pos - q_curr);

    //normalize q vel zero for redundancy
    if (q_dot_zero.norm() > joint_max_velocity){
        q_dot_zero = joint_max_velocity * (q_dot_zero / q_dot_zero.norm());
    }

    //kinematic inverse
    Eigen::VectorXd task_ang_vel_ref = k_att * pose_err_vec_world.tail<3>();

    //normalize joint vel
    if (task_ang_vel_ref.norm() > angular_max_velocity){
        task_ang_vel_ref = angular_max_velocity * (task_ang_vel_ref / task_ang_vel_ref.norm());
    }


    //6x1
    Eigen::VectorXd task_vel_ref;
    task_vel_ref.resize(6);
    task_vel_ref.head<3>() = x_vel_ref;
    task_vel_ref.tail<3>() = task_ang_vel_ref;

                  
    Eigen::VectorXd desired_joint_velocity = pseudoInverse * task_vel_ref + nullspace * q_dot_zero ;

    //normalize joint vel
    if (desired_joint_velocity.norm() > joint_max_velocity){
        desired_joint_velocity = joint_max_velocity * (desired_joint_velocity / desired_joint_velocity.norm());
    }

    Eigen::VectorXd desired_joint_pos;
    desired_joint_pos.resize(7);

    desired_joint_pos = q_curr + 10*(desired_joint_velocity)*(1.0/publish_rate_param);

    //float array that will carry the desired joint position
    command.data.clear();
    for (int i = 0; i < t_joint_positions.size(); ++i){
        command.data.push_back (desired_joint_pos[i]);
    }

    Eigen::Quaterniond q_ini_;
    Eigen::Quaterniond q_tar_;
    
    //check ori difference
    // //get fbk quaternion
    // pinocchio::quaternion::assignQuaternion(q_ini_, data_.oMi[7].rotation());

    // //convert euler target angles to rotation and then to quaternion
    // Eigen::MatrixXd tar_rot_matrix = pinocchio::rpy::rpyToMatrix(target_roll,target_pitch, target_yaw);
    // pinocchio::quaternion::assignQuaternion(q_tar_, tar_rot_matrix);
    // // Assign values to q_target and q_feedback

    // // Calculate dot product
    // double dot = q_tar_.dot(q_ini_);

    // // Calculate the angle between quaternions
    // double theta = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot))); // Ensure dot is within [-1, 1]

    // ROS_INFO("%f", theta);

    publisher.publish(command);


    //////////////////////

    //send feedback pose and twist
    geometry_msgs::Pose fbk_pose;
    geometry_msgs::Twist fbk_twist;


    Eigen::Quaterniond q_ini;        
    //get fbk quaternion
    pinocchio::quaternion::assignQuaternion(q_ini, data_.oMi[7].rotation());

    fbk_pose.position.x = curr_pos[0];
    fbk_pose.position.y = curr_pos[1];
    fbk_pose.position.z = curr_pos[2];

    fbk_pose.orientation.x = q_ini.x();
    fbk_pose.orientation.y = q_ini.y();
    fbk_pose.orientation.z = q_ini.z();
    fbk_pose.orientation.w = q_ini.w();

    fbk_twist.linear.x = task_vel_ref[0];
    fbk_twist.linear.y = task_vel_ref[1];
    fbk_twist.linear.z = task_vel_ref[2];
    fbk_twist.angular.x = task_vel_ref[3];
    fbk_twist.angular.y = task_vel_ref[4];
    fbk_twist.angular.z = task_vel_ref[5];

    pose_publisher.publish(fbk_pose);
    twist_publisher.publish(fbk_twist);

}

