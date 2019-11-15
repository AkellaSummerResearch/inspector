

#include <mission_planner/collision_avoidance.h>

// ---------------------------------------------------
namespace inspector {
	
void InspectorClass::Mission(ros::NodeHandle *nh) {
    nh_ = *nh;

    // Get namespace of current node
    nh_.getParam("namespace", ns_);
    ROS_INFO("[mission_node] namespace: %s", ns_.c_str());

    // Get update rate for tf threads
    double tf_update_rate;
    nh_.getParam("tf_update_rate", tf_update_rate);

    // Get guidance parameters
    double takeoff_height;
    nh_.getParam("takeoff_height", takeoff_height);
    nh_.getParam("max_acceleration", max_acceleration_);
    nh_.getParam("max_velocity", max_velocity_);
    nh_.getParam("max_yaw_vel", max_yaw_vel_);
    max_yaw_vel_ = M_PI/8;
    max_acceleration_ = 2.5;

    // Get cam position in body frame
    std::vector<double> final_point;
    nh_.getParam("final_waypoint", final_point);
    Eigen::Vector3d des_point(final_point[0], final_point[1], final_point[2]);

    // Start the Mission Planner Engine
    ROS_INFO("[mission_node] Starting Mission Planner Engine!");    
    mission_.Initialize(ns_, tf_update_rate, max_velocity_);

    // Start waypoint marker publisher and delete all markers currently published
    ROS_INFO("[mission_node] Creating visualization marker publisher!");    
    waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("MissionWps", 2, true);
    visualization_msgs::MarkerArray wp_markers;
    visualization_functions::DeleteMarkersTemplate(markers_frame_id_, &wp_markers);
    waypoint_marker_pub_.publish(wp_markers);

    // Wait until measurements are available
    ROS_INFO("[mission_node] Waiting for first pose in tf tree...");
    tf::StampedTransform tf_initial_pose = mission_.WaitForFirstPose();
    mission_planner::xyz_heading origin(tf_initial_pose);
    ROS_INFO("[mission_node] First pose obtained from tf tree!");

    // Navigation constant variables
    const double max_vel = max_velocity_, max_acc = 5.0;
    const Eigen::Vector3d init_vel(0.0, 0.0, 0.0), final_vel(0.0, 0.0, 0.0);
    const double sampling_time = 0.01;

    // Takeoff
    mission_planner::xyz_heading final_waypoint;     // Variable used to save last waypoint on each sequence of waypoints
    ROS_INFO("[mission_node] Taking off!");
    mission_.Takeoff(ns_, takeoff_height-origin.z_, sampling_time, max_velocity_, nh, &final_waypoint);

    // Wait until quad is done taking off before executing the next step of the mission
    ROS_INFO("[mission_node] Wait until quad is idle...");
    mission_.ReturnWhenIdle();
    ROS_INFO("[mission_node] Quad is idle!"); 

    // Plan to go to desired waypoint
    // Eigen::Vector3d des_point = Eigen::Vector3d(3.0, 0.0, 0.0);
    std::string service_name = "/" + ns_ + "/mapper_node/rrg";
    ros::ServiceClient rrg_client = nh->serviceClient<mapper::RRT_RRG_PRM>(service_name);
    
    if(!rrg_client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[%s mission_node] Service ""%s"" unavailable for call.", ns_.c_str(), rrg_client.getService().c_str());
    }

    // Find trajectory around obstacle
    mapper::RRT_RRG_PRM path_plan_msg;
    path_plan_msg.request.max_time = 0.5;
    path_plan_msg.request.max_nodes = 8000;
    path_plan_msg.request.steer_param = 0.25;
    path_plan_msg.request.free_space_only = false;
    path_plan_msg.request.origin = final_waypoint.GetXYZ();
    path_plan_msg.request.destination = helper::eigenvec2rospoint(des_point);
    path_plan_msg.request.box_min = helper::set_rospoint(-5, -3, 0.3);
    path_plan_msg.request.box_max = helper::set_rospoint( 5,  3, 1.5);
    path_plan_msg.request.prune_result = true;
    path_plan_msg.request.publish_rviz = true;
    if (rrg_client.call(path_plan_msg)) {
        if(path_plan_msg.response.success == false) {
            ROS_ERROR("[%s mission_node] Path Planner could not find a feasible solution.", ns_.c_str());

            // Land
            std::vector<mission_planner::xyz_heading> waypoints;
            waypoints.clear();
            waypoints.push_back(final_waypoint);
            waypoints.push_back(mission_planner::xyz_heading(final_waypoint.x_, final_waypoint.y_, -0.5, final_waypoint.yaw_));
            std::string name = ns_ + "/land";
            mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, name, &final_waypoint);

            // Wait until quad is done with landing before executing the next step of the mission
            ROS_INFO("[mission_node] Wait until quad is idle...");
            mission_.ReturnWhenIdle();
            ROS_INFO("[mission_node] Quad is idle!");

            // Disarm quad
            mission_.DisarmQuad(ns_, nh);

            return;
        }
    } else {
        ROS_ERROR("[%s mission_node] Failed to call path planning service %s.",
                  ns_.c_str(), rrg_client.getService().c_str());
    
        // Land
        std::vector<mission_planner::xyz_heading> waypoints;
        waypoints.clear();
        waypoints.push_back(final_waypoint);
        waypoints.push_back(mission_planner::xyz_heading(final_waypoint.x_, final_waypoint.y_, -0.5, final_waypoint.yaw_));
        std::string name = ns_ + "/land";
        mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, name, &final_waypoint);

        // Wait until quad is done with landing before executing the next step of the mission
            ROS_INFO("[mission_node] Wait until quad is idle...");
        mission_.ReturnWhenIdle();
        ROS_INFO("[mission_node] Quad is idle!");

        // Disarm quad
        mission_.DisarmQuad(ns_, nh);

        return;
    }
    ROS_INFO("Planning time: %f", path_plan_msg.response.planning_time);
    ROS_INFO("Planning nodes: %d", path_plan_msg.response.n_nodes);

    std::vector<mission_planner::xyz_heading> waypoint_list;
    ros::ServiceClient min_time_client = nh->serviceClient<p4_ros::min_time>("/p4_services/min_time_solver");
    p4_ros::min_time min_time_req;
    for (uint i = 0; i < path_plan_msg.response.path.size(); i++) {
        Eigen::Vector3d pt = helper::rospoint2eigenvec(path_plan_msg.response.path[i]);
        std::cout << pt.transpose() << std::endl;
        min_time_req.request.pos_array.push_back(path_plan_msg.response.path[i]);
        waypoint_list.push_back(mission_planner::xyz_heading(pt, origin.yaw_));
    }
    std::cout << std::endl;
    mission_.ReturnWhenIdle();

    // Publish inspection waypoint markers
    this->PublishWaypointMarkers(waypoint_list);

    // Plan minimum time trajectory
    min_time_req.request.sampling_freq = 30;
    min_time_req.request.corridor_width = 0.1;
    min_time_req.request.max_vel = max_velocity_;
    min_time_req.request.max_acc = max_acceleration_;
    min_time_req.request.max_jerk = 1.0;
    min_time_req.request.visualize_output = false;
    ros::Time t0 = ros::Time::now();
    ROS_INFO("[p4_services] Calling service %s!", min_time_client.getService().c_str());
    if (!min_time_client.call(min_time_req)) {
        ROS_WARN("Error when calling the trajectory optimizer!");
    }
    ros::Time t1 = ros::Time::now();
    ROS_INFO("Solution time: %f", (t1 - t0).toSec());

    if(min_time_req.response.final_time <= 0) {
        ROS_WARN("Solution failed!");
    } else {
        mission_planner::TrajectoryActionInputs traj_inputs;
        traj_inputs.start_immediately = false;
        traj_inputs.sampling_time = 1.0/min_time_req.request.sampling_freq;
        traj_inputs.action_type = mission_planner::ActionType::Trajectory;
        traj_inputs.flatStates = helper::min_time_to_flat_states(min_time_req.response.pva_vec, origin.yaw_);

        mission_.AddTrajectoryToBuffer(traj_inputs);
        mission_.AddToRvizBuffer(waypoint_list, traj_inputs.flatStates, "trajectory");
        final_waypoint = mission_planner::xyz_heading(des_point, origin.yaw_);
    }

    // ROS_INFO("Adding waypoints into buffer.");
    // mission_.AddWaypoints2Buffer(waypoint_list, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);


    // Land
    std::vector<mission_planner::xyz_heading> waypoints;
    waypoints.clear();
    waypoints.push_back(final_waypoint);
    waypoints.push_back(mission_planner::xyz_heading(final_waypoint.x_, final_waypoint.y_, -0.5, final_waypoint.yaw_));
    std::string name = ns_ + "/land";
    mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, name, &final_waypoint);

    // // Send waypoints to path planner
    // waypoints.clear();
    // waypoints.push_back(final_waypoint);
    // waypoints.push_back(mission_planner::xyz_heading(des_point, origin.yaw_));
    // mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);

    // Wait until quad is done with landing before executing the next step of the mission
    ROS_INFO("[mission_node] Wait until quad is idle...");
    mission_.ReturnWhenIdle();
    ROS_INFO("[mission_node] Quad is idle!");

    // Disarm quad
    mission_.DisarmQuad(ns_, nh);

}

void InspectorClass::PublishWaypointMarkers(const std::vector<mission_planner::xyz_heading> &waypoint_list) {
    // Set marker properties
    double diameter = 0.01;
    std_msgs::ColorRGBA color = visualization_functions::Color::Cyan();
    std::string ns = "waypoints";
    double marker_length = 0.1;
    visualization_msgs::MarkerArray wp_markers;

    for (uint i = 0; i < waypoint_list.size(); i++) {
        // Get initial and final points
        double yaw = waypoint_list[i].GetYaw();
        Eigen::Vector3d p1 = waypoint_list[i].GetEigenXYZ();
        Eigen::Vector3d p2 = p1 + marker_length*Eigen::Vector3d(cos(yaw), sin(yaw), 0.0);
        
        // Get visualization marker for the waypoint
        visualization_msgs::Marker waypoint;
        visualization_functions::DrawArrowPoints(p1, p2, color, markers_frame_id_, ns, i, 
                                                 diameter, &waypoint);
        wp_markers.markers.push_back(waypoint);
    }

    // Publish all waypoints
    waypoint_marker_pub_.publish(wp_markers);
    ROS_INFO("Published into topic: %s", waypoint_marker_pub_.getTopic().c_str());
}

} // inspector