

// Standard includes
#include <ros/ros.h>
#include <mission_planner/vicon_swarm_inspection.h>

int main(int argc, char **argv) {

	ROS_INFO("[swarm_mission_node]: Starting...");

	ros::init(argc, argv, "mission_node");
	ros::NodeHandle node("~");
	
	inspector::InspectorClass mission_obj;
	mission_obj.Mission(&node);

    // Gollum path planning example ----------------------------------------------------------
	std::string ns = "Gollum";
	double tf_update_rate = 10;
	double max_velocity = 0.20;
	double max_acc = 5;
	uint rover_index = 2;
	rover_planner::RoverMissionClass rover(ns, tf_update_rate, max_velocity, rover_index);

    // Wait until measurements are available
    ROS_INFO("[mission_node] Waiting for first pose in tf tree...");
    tf::StampedTransform tf_initial_pose = rover.WaitForFirstPose();
    Eigen::Vector2d origin = Eigen::Vector2d(tf_initial_pose.getOrigin().x(),
    	                                     tf_initial_pose.getOrigin().y());
    ROS_INFO("[mission_node] First pose obtained from tf tree!");
	
	std::vector<Eigen::Vector2d> wps;
	wps.push_back(origin);
	
	wps.push_back(origin + Eigen::Vector2d(0.7, -0.3));
	wps.push_back(origin + Eigen::Vector2d(0.7, -1.5));
	wps.push_back(origin + Eigen::Vector2d(0.0, -1.8));
	wps.push_back(origin + Eigen::Vector2d(-0.7, -1.5));
	wps.push_back(origin + Eigen::Vector2d(-0.7, -0.3));
	//wps.push_back(origin + Eigen::Vector2d(-0.5, 0.0));
	//wps.push_back(origin + Eigen::Vector2d(0.0, 0.8));
	//wps.push_back(origin + Eigen::Vector2d(1.6, 0.8));
	//wps.push_back(origin + Eigen::Vector2d(1.6, -0.8));
	//wps.push_back(origin + Eigen::Vector2d(0.0, -0.8));
	wps.push_back(origin);
	Eigen::Vector2d init_vel = Eigen::Vector2d(0.0, 0.0);
	Eigen::Vector2d final_vel = Eigen::Vector2d(0.0, 0.0);
	Eigen::Vector2d final_wp;

	ros::Duration(1.0).sleep();
	rover.AddWaypoints2Buffer(wps, init_vel, final_vel, max_velocity, max_acc, "traj", &final_wp);

	//rover.ReturnWhenIdle();
	//ROS_INFO("Rover is idle");

	// Mapper planning example ----------------------------------------------------------------
	

	ros::spin();

	return 0;
}