#include "kinematic_controller/joint.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_kinematic_controller");
	ros::NodeHandle nh("~");

    ros::Time::waitForValid();
    
    KinematicJointController controller(nh);

    ros::Rate publish_rate(controller.publish_rate_param);
    

    while (ros::ok) {
		
		ros::spinOnce() ;

        controller.update();
    
        publish_rate.sleep() ;

	}

	return 0;
}
