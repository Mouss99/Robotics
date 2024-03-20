#include "kinematic_controller/task.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "task_controller");
	ros::NodeHandle nh("~");

    KinematicTaskController controller(nh);

    ros::Rate publish_rate(controller.publish_rate_param);

    ros::ServiceServer move_service = nh.advertiseService("/pose_planner/move_to", &KinematicTaskController::move_service_call_back, &controller);
    ros::ServiceServer interpolate_service = nh.advertiseService("/pose_planner/move_ori", &KinematicTaskController::interpolation_service_call_back, &controller);
    ros::Time::waitForValid();

    while (ros::ok() && !controller.ready) {
        ros::spinOnce();
        publish_rate.sleep();
    }

     while (ros::ok()) {

        ros::spinOnce();
        
        controller.update();

        publish_rate.sleep();

	}

	return 0;
}
