

// Standard includes
#include <ros/ros.h>
#include <mission_planner/mapper_inspection_class.h>

int main(int argc, char **argv) {

	ROS_INFO("[mission_node]: Starting...");

	ros::init(argc, argv, "mission_node");
	ros::NodeHandle node("~");
	
	inspector::InspectorClass mission_obj;
	mission_obj.Mission(&node);

	ros::spin();

	return 0;
}