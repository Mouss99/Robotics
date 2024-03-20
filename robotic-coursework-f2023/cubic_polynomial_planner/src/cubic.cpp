#include <ros/ros.h>
#include <highlevel_msgs/MoveTo.h>
#include <geometry_msgs/Pose.h>

//global variables
geometry_msgs::Pose current_pose;
geometry_msgs::Pose desired_pose;
bool service_called;
double start_time;

float target_x;
float target_y;
float target_z;
float target_T;

// Setup the publisher
ros::Publisher publisher;


// Callback function for the robot pose subscriber
void pose_call_back(const geometry_msgs::Pose::ConstPtr& msg) {
    current_pose = *msg;
}


bool my_call_back (highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res) {

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
    start_time = ros::Time::now().toSec();
    return true;

}

void update (){

    if (service_called){
        //calculate t
        double current_time = ros::Time::now().toSec();
        double t = current_time - start_time;
        
        //calculate desired position
        desired_pose.position.x = current_pose.position.x + (((3 * pow(t, 2)) / pow(target_T, 2)) - ((2 * pow(t, 3)) / pow(target_T, 3))) * (target_x - current_pose.position.x);
        desired_pose.position.y = current_pose.position.y + (((3 * pow(t, 2)) / pow(target_T, 2)) - ((2 * pow(t, 3)) / pow(target_T, 3))) * (target_y - current_pose.position.y);
        desired_pose.position.z = current_pose.position.z + (((3 * pow(t, 2)) / pow(target_T, 2)) - ((2 * pow(t, 3)) / pow(target_T, 3))) * (target_z - current_pose.position.z);

        //prints for testing
        //printf("My x is: %f\n", desired_pose.position.x);
        //printf("My y is: %f\n", desired_pose.position.y);
        //printf("My z is: %f\n", desired_pose.position.z);
        //double test_time = ros::Time::now().toSec();
        //double test = test_time - start_time;
        //printf("Current ROS Time: %.9f\n",test);
        if (t >= target_T){
            desired_pose.position.x = target_x;
            desired_pose.position.y = target_y;
            desired_pose.position.z = target_z;
            service_called = false;
        }
    }

    else{
        desired_pose.position.x = target_x;
        desired_pose.position.y = target_y;
        desired_pose.position.z = target_z;
    }
    publisher.publish(desired_pose);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "server");
    ros::NodeHandle nh;
    
    // Subscribe to the robot pose topic
    ros::Subscriber pose_subscriber = nh.subscribe("/firefly/ground_truth/pose", 1, pose_call_back);

    // Advertise the "move_to" service
    ros::ServiceServer service = nh.advertiseService("/pose_planner/move_to", my_call_back);

    publisher = nh.advertise<geometry_msgs::Pose>("/firefly/command/pose", 1);

    // Set the control loop frequency
    ros::Rate loop_rate(500);  // 500Hz

    while (ros::ok()) {
        // Your control code here

        ros::spinOnce(); 
        
        //call the publish function that will do the computations if service call is correct and then publish the results
        update();

        loop_rate.sleep();
    }

    return 0;
}
