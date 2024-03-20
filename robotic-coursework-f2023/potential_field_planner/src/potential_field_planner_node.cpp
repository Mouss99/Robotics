#include <ros/ros.h>
#include "potential_field_planner/PotentialFieldPlanner.hpp"


int main(int argc, char** argv) {

    // Initialize ROS
	ros::init(argc, argv, "hello_parameter");

	// Add a node handle
	ros::NodeHandle nh("~");

	PotentialFieldPlanner planner(nh);
   
	ros::ServiceServer service = nh.advertiseService("start", &PotentialFieldPlanner::start, &planner);

	ros::Rate loopRate(500);
	while (ros::ok) {
		
		// call all the callbacks waiting to be called
		ros::spinOnce() ;

		planner.publish_all();

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;
	}

	return 0;
}