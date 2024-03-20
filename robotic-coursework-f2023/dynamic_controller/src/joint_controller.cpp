#include "dynamic_controller/joint.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_dynamic_controller");
	ros::NodeHandle nh("~");
    
    DynamicJointController controller(nh);

    ros::Rate publish_rate(controller.publish_rate_param);

    ros::Time::waitForValid();

    ros::spinOnce() ;

    while (ros::ok) {
		
		ros::spinOnce() ;

        controller.update();
    
        publish_rate.sleep() ;

	}

	return 0;
}