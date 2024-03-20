#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

//publish to husky velocity 
ros::Publisher publisher;
   
void call_back_function(const std_msgs::String::ConstPtr& msg) {

    /// Process the received letter here
    std::string user_input = msg->data;

    geometry_msgs::Twist twist_cmd;
    twist_cmd.linear.x = 0.0;
    twist_cmd.angular.z = 0.0;


    if (user_input == "i")
    {
    twist_cmd.linear.x = 0.5;
    }
    else if (user_input == "u")
    {
    twist_cmd.linear.x = 0.5;
    twist_cmd.angular.z = 0.5;
    }
    else if (user_input == "o")
    {
    twist_cmd.linear.x = 0.5;
    twist_cmd.angular.z = -0.5;
    }
        
    publisher.publish(twist_cmd);
    }


int main(int argc, char **argv) {

    //initialize ROS 
    ros::init(argc, argv, "subscriber_node");

    //create node handle
    ros::NodeHandle node_handle;

    //specify the frequency to 10HZ
    ros::Rate loopRate(10);

    //listening to topic "teleop/cmd"
    ros::Subscriber subscriber = node_handle.subscribe("teleop/cmd",1, call_back_function);

    // Create a publisher to send twist commands to control the robot
    publisher = node_handle.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);


    //while program is running without interruption
    while (ros::ok())
    {
        // Do other processing or add a sleep if needed
        ros::spinOnce();
        loopRate.sleep();
        
    }

    return 0;
}




