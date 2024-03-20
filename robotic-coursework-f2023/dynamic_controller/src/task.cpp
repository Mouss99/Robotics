#include "dynamic_controller/task.hpp"


DynamicTaskController::DynamicTaskController(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {

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

    subscriber = nh.subscribe("/gen3/joint_states", 1, &DynamicTaskController::subscriber_call_back, this);

    current_joint_pos.resize(7);
    current_joint_vel.resize(7);

    std::fill(current_joint_pos.begin(), current_joint_pos.end(), 0);
    std::fill(current_joint_vel.begin(), current_joint_vel.end(), 0);

    x_pos_ref.resize(3);
    x_vel_ref.resize(3);
    curr_pos.resize(6);
    // ori_ref.resize(3);

    default_pos = true;
    default_ori = true;
    service_called = false;
}


bool DynamicTaskController::readParameters() {
    
  
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

void DynamicTaskController::subscriber_call_back(const sensor_msgs::JointState::ConstPtr& msg){

    current_joint_pos = msg -> position;
    current_joint_vel = msg -> velocity;

}

bool DynamicTaskController::move_service_call_back (highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res) {

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

bool DynamicTaskController::interpolation_service_call_back (highlevel_msgs::MoveOri::Request &req, highlevel_msgs::MoveOri::Response &res) {

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



void DynamicTaskController::forward_kinematics(){ 
    
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

Eigen::MatrixXd DynamicTaskController::computeJacobian(){
    const int JOINT_ID = 7;
    int dim_joints = model_.nq ;

    Eigen::MatrixXd jacobian_ = Eigen::MatrixXd::Zero(6,dim_joints) ;
    
    //get current joint pos and map it to eigen 
    Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());

    Eigen::VectorXd joint_vel = Eigen::Map<Eigen::VectorXd>(current_joint_vel.data(), current_joint_vel.size());

    pinocchio::computeAllTerms(model_, data_, joint_pos, joint_vel) ;

    // Ensure you have model and joint_pos properly defined.
    pinocchio::getJointJacobian(model_, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);
    
    // Extract a 3x7 submatrix
    //Eigen::Matrix<double, 3, 7> submatrix_jacobian = jacobian_.block<3, 7>(0, 0);
    return jacobian_;

}

Eigen::MatrixXd DynamicTaskController::computeJacobianDot(){
    const int JOINT_ID = 7;
    int dim_joints = model_.nq ;

    Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(6,dim_joints) ;
    
    //get current joint pos and map it to eigen 
    Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());

    Eigen::VectorXd joint_vel = Eigen::Map<Eigen::VectorXd>(current_joint_vel.data(), current_joint_vel.size());

    pinocchio::computeAllTerms(model_, data_, joint_pos, joint_vel) ;

    // Ensure you have model and joint_pos properly defined.
    pinocchio::getJointJacobianTimeVariation(model_, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot);
    
    // Extract a 3x7 submatrix
    //Eigen::Matrix<double, 3, 7> submatrix_jacobian = jacobian_dot.block<3, 7>(0, 0);
    return jacobian_dot;

}

std::vector<Eigen::VectorXd> DynamicTaskController::plan(){

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
            q_pos[i] = (qc + 300*q_vel[i]*(1.0/publish_rate_param));

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



void DynamicTaskController::update(){

        forward_kinematics();
        Eigen::VectorXd targ_pos = Eigen::Map<Eigen::VectorXd>(t_hand_positions.data(), t_hand_positions.size());
        Eigen::VectorXd targ_ori = Eigen::Map<Eigen::VectorXd>(t_hand_ori.data(), t_hand_ori.size());

        //if service call, use cubic polynomial
        if (service_called){
            //calculate t
            double current_time = ros::Time::now().toSec();
            double t = current_time - start_time;
            //loop from 0->T
            if (t < target_T){

                 Eigen::Vector3d serv_targ;
                 serv_targ << target_x, target_y, target_z; 

                //desired_position
                x_pos_ref =  (curr_pos.head<3>()+ (((3 * std::pow(t, 2)) / std::pow(target_T, 2)) - ((2 * std::pow(t, 3)) / std::pow(target_T, 3))) * (serv_targ - curr_pos.head<3>()));
                
                //desired velocity
                //x_vel_ref = (((6*t) / std::pow(target_T, 2)) - ((6 * std::pow(t, 2)) / std::pow(target_T, 3))) * (serv_targ - curr_pos.head<3>());
                x_vel_ref = 8 * (targ_pos - curr_pos.head<3>());

                //normalize linear vel
                if (x_vel_ref.norm() > linear_max_velocity){
                    x_vel_ref = linear_max_velocity * (x_vel_ref / x_vel_ref.norm());
                }
                }
            
            else{
                service_called = false;
            }
        }
        //if trying to send arm to default position, use the potential field
        if (default_pos){

            x_vel_ref = 8 * (targ_pos - curr_pos.head<3>());

            //normalize linear vel
            if (x_vel_ref.norm() > linear_max_velocity){
                x_vel_ref = linear_max_velocity * (x_vel_ref / x_vel_ref.norm());
            }
            x_pos_ref = curr_pos.head<3>() + (x_vel_ref/publish_rate_param);
        }

        if (ori_service_called){
            //calculate t
            double current_time = ros::Time::now().toSec();
            double t = current_time - ori_start_time;

            Eigen::Quaterniond q_ini;
            Eigen::Quaterniond q_tar;
            Eigen::Quaterniond q_s;
            
            //get fbk quaternion
            pinocchio::quaternion::assignQuaternion(q_ini, data_.oMi[7].rotation());

            //convert euler target angles to rotation and then to quaternion
            Eigen::MatrixXd tar_rot_matrix = pinocchio::rpy::rpyToMatrix(target_roll,target_pitch, target_yaw);
            pinocchio::quaternion::assignQuaternion(q_tar, tar_rot_matrix);

            //loop from 0->T
            if (t < target_T_ori){

                double s = (((3 * std::pow(t, 2)) / std::pow(target_T_ori, 2)) - ((2 * std::pow(t, 3)) / std::pow(target_T_ori, 3)));

                q_tar.normalize();
                
                q_s = q_ini.slerp(s, q_tar);
                
                 // Convert quaternion to rotation matrix
                Eigen::MatrixXd ref_rot_matrix = q_s.toRotationMatrix();


                pinocchio::SE3 temp;
                temp.rotation() = ref_rot_matrix;

                //convert rot matrix to euler angle
                ori_ref = pinocchio::rpy::matrixToRpy(temp.rotation());

                }
            
            else{
                ori_service_called = false;
            }
        }

        if(default_ori){
            //Orientation
            //Get fbk and ref pose for translation and orientation
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

            ori_ref = pose_err_vec_world.tail<3>();

        }

        //add euler angles and angular velocities to computations
        x_pos_ref.conservativeResize(6);
        x_pos_ref.tail<3>() = ori_ref;

        x_vel_ref.conservativeResize(6);
        Eigen::Vector3d ang_vel(0, 0, 0);
        x_vel_ref.tail<3>() = ang_vel;


        //compute jacobian
        Eigen::MatrixXd jacobian = computeJacobian();

        //compute pseudo inverse
        Eigen::MatrixXd j_pseudo = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd jT_pseudo = (jacobian.transpose()).completeOrthogonalDecomposition().pseudoInverse();

        //compute all terms in pinocchio
        Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(current_joint_pos.data(), current_joint_pos.size());
        Eigen::VectorXd joint_vel = Eigen::Map<Eigen::VectorXd>(current_joint_vel.data(), current_joint_vel.size());
        pinocchio::computeAllTerms(model_, data_, joint_pos, joint_vel);

        //use pinocchio to get M and h
        //7x1
        Eigen::VectorXd h = data_.nle;
        //7x7
        Eigen::MatrixXd M = data_.M;

        //calculate lambda
        Eigen::MatrixXd lambda = jT_pseudo*M*j_pseudo;
                        
        //jacobian dot
        Eigen::MatrixXd jacobian_dot = computeJacobianDot();

        //calculate eta
        Eigen::MatrixXd eta = jT_pseudo*h - lambda*jacobian_dot*joint_vel;

        //getting joint values for redundancy
        //identity matrix for redundancy
        Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(7, 7);
        std::vector<Eigen::VectorXd> potential_functions = plan();

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
                
            //seperate K and D for redundancy
            int K_ = 6;
            int D_ = 4;

            q_double_dot[i] = q_acc[i] + D_*(q_vel[i] - current_joint_vel[i]) + K_*(q_pos[i] - current_joint_pos[i]);

        }

        Eigen::VectorXd x_double_dot;
        x_double_dot.resize(6);

        //damping and stiffness
        Eigen::MatrixXd K = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(6, 6);

        K.diagonal() << 65,65,65,3,3,3;
        D.diagonal() << 5,5,5,1,1,1;

        //get current task velocity using kinematics
        x_vel_fbk = jacobian*joint_vel;

        //get desired task acceleration
        Eigen::VectorXd x_acc_ref = (x_vel_ref - x_vel_fbk)/publish_rate_param;

        x_double_dot = x_acc_ref + D* (x_vel_ref - x_vel_fbk) + K * (x_pos_ref - curr_pos);

        Eigen::VectorXd F = lambda* x_double_dot + eta;

        //add redundancy
        Eigen::MatrixXd P = identity_matrix - jacobian.transpose() * ((jacobian*M.inverse()*jacobian.transpose())).inverse() * jacobian * M.inverse();

        Eigen::VectorXd tau_zero = M*q_double_dot + h;

        //final result
        Eigen::VectorXd torque = (jacobian.transpose())*F + P*tau_zero;

        Eigen::Quaterniond q_ini;
            Eigen::Quaterniond q_tar;
            
            //get fbk quaternion
            pinocchio::quaternion::assignQuaternion(q_ini, data_.oMi[7].rotation());

            //convert euler target angles to rotation and then to quaternion
            Eigen::MatrixXd tar_rot_matrix = pinocchio::rpy::rpyToMatrix(target_roll,target_pitch, target_yaw);
            pinocchio::quaternion::assignQuaternion(q_tar, tar_rot_matrix);
// Assign values to q_target and q_feedback

// Calculate dot product
double dot = q_tar.dot(q_ini);

// Calculate the angle between quaternions
double theta = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot))); // Ensure dot is within [-1, 1]

ROS_INFO("%f", theta);

        command.data.clear();
        for (int i = 0; i < 7; i++){
            command.data.push_back(torque[i]);
        }

        publisher.publish(command);

}
