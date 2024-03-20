#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


geometry_msgs::Pose desired_pose;
geometry_msgs::Pose feedback_pose;
geometry_msgs::Twist desired_twist;

geometry_msgs::Twist target_twist;


bool is_done;

void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
    desired_twist = *twist_msg;

 }

void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg) {
    desired_pose = *pose_msg;
}

void doneCallback(const std_msgs::Bool::ConstPtr& done_msg) {
    is_done = done_msg->data;
   }

 void feedbackPoseCallback(const geometry_msgs::Pose::ConstPtr& feedback_msg) {
    feedback_pose = *feedback_msg;
 }

 //Normalize
double normalize(double a_vel, double max_vel){
        return (a_vel / std::abs(a_vel)) * max_vel;
}

 void calculateAngularVelocity(geometry_msgs::Pose c_pose, geometry_msgs::Pose d_pose, geometry_msgs::Twist d_twist, double lin_max_vel, double ang_max_vel){

        //d should be target pose
	double x_r = d_pose.position.x - c_pose.position.x;
	double y_r = d_pose.position.y - c_pose.position.y;

    tf2::Quaternion quaternion;

    tf2::fromMsg(c_pose.orientation, quaternion);

    tf2::Matrix3x3 rot_matrix;
    rot_matrix.setRotation(quaternion);

    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);

	Eigen::Matrix2d jacobian;
	jacobian << cos(yaw), -sin(yaw)*x_r-cos(yaw)*y_r, 
                    sin(yaw), cos(yaw)*x_r-sin(yaw)*y_r;

	//inverse the jacobian
	Eigen::Matrix2d inverse_jacobian = jacobian.inverse();

	Eigen::Vector2d linear_vel;

	linear_vel << d_twist.linear.x,d_twist.linear.y;


    // Calculate inverse Jacobian and compute joint velocities
    Eigen::Vector2d result = inverse_jacobian * linear_vel;

    // Extract linear and angular velocities from the result and assign them to new_twist
    target_twist.linear.x = result(0);

    if (std::abs(target_twist.linear.x) > lin_max_vel){
        target_twist.linear.x = normalize(target_twist.linear.x, lin_max_vel);
    }

    target_twist.angular.z = result(1);
    
    if (std::abs(target_twist.angular.z) > ang_max_vel){
        target_twist.angular.z = normalize(target_twist.angular.z, ang_max_vel);
    }


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;


    ros::Subscriber twist_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber done_sub;
    ros::Subscriber feedback_sub;

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);

    // Initialize subscribers
    twist_sub = nh.subscribe("/planner/twist", 1, twistCallback);
    pose_sub = nh.subscribe("/planner/pose", 1, poseCallback);
    done_sub = nh.subscribe("/planner/done", 1, doneCallback);
    feedback_sub = nh.subscribe("/husky_velocity_controller/feedback/pose", 1, feedbackPoseCallback);
    // reading the local parameter

    double target_x;
    double target_y;
    double linear_max_velocity_;
    double angular_max_velocity_;

    if (!nh.getParam("/target/x", target_x)) {
        ROS_INFO("Cannot read parameter x");
    }

    // reading the local parameter
    if (!nh.getParam("/target/y", target_y)) {
        ROS_INFO("Cannot read parameter y");
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

    geometry_msgs::Pose target_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;

    ros::Rate loopRate(500);  // 500Hz
    while (ros::ok())
    {   
        calculateAngularVelocity(feedback_pose,target_pose,desired_twist,linear_max_velocity_,angular_max_velocity_);
         if (is_done){
            target_twist.linear.x = 0;
            target_twist.angular.z = 0;
         }
        
        twist_pub.publish(target_twist);
        
        // Do other processing or add a sleep if needed
        ros::spinOnce();
        loopRate.sleep();
        
    }

    return 0;

}
