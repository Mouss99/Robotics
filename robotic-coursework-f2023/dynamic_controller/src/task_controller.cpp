#include "dynamic_controller/task.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_task_controller");
	ros::NodeHandle nh("~");

    DynamicTaskController controller(nh);

    ros::Rate publish_rate(controller.publish_rate_param);

    ros::ServiceServer move_service = nh.advertiseService("/pose_planner/move_to", &DynamicTaskController::move_service_call_back, &controller);
    ros::ServiceServer interpolate_service = nh.advertiseService("/pose_planner/move_ori", &DynamicTaskController::interpolation_service_call_back, &controller);
    
    
    ros::Time::waitForValid();

    while (ros::ok()) {

        ros::spinOnce();
        
        controller.update();

        publish_rate.sleep();

	}

	return 0;
}
