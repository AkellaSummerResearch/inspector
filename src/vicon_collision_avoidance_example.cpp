

#include <mission_planner/vicon_collision_avoidance_example.h>

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
    nh_.getParam("max_velocity", max_velocity_);

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

    // Variable for setting waypoints
    std::vector<mission_planner::xyz_heading> waypoints;
    mission_planner::xyz_heading final_waypoint;     // Variable used to save last waypoint on each sequence of waypoints

    // Navigation constant variables
    const double max_vel = max_velocity_, max_acc = 5.0;
    const Eigen::Vector3d init_vel(0.0, 0.0, 0.0), final_vel(0.0, 0.0, 0.0);
    const double sampling_time = 0.01;

    // Takeoff
    ROS_INFO("[mission_node] Taking off!");
    mission_.Takeoff(ns_, takeoff_height-origin.z_, sampling_time, max_velocity_, nh, &final_waypoint);

    // Wait until quad is done taking off before executing the next step of the mission
    ROS_INFO("[mission_node] Wait until quad is idle...");
    mission_.ReturnWhenIdle();
    ROS_INFO("[mission_node] Quad is idle!"); 

    // Start service for finding relative pose between SLAM and vicon frames
    geometry_msgs::Pose rel_pose;
    if (!this->StartRelPoseEstimator(&rel_pose)) {
        return;
    }

    // Create thread that publishes relative tf between vicon and slam frames
    ROS_INFO("[mission_node] Publishing relative pose between vicon and slam frames!");
    rel_tf_pub_thread_ = std::thread(&InspectorClass::RelTfPubTask, this, rel_pose);

    // Load octomap in slam frame
    this->LoadOctomap();

    // Plan to go to desired waypoint
    Eigen::Vector3d des_point = this->Slam2Vicon(Eigen::Vector3d(1.75, 0.3, 0.20));
    // std::cout << des_point.transpose() << std::endl;
    std::string service_name = "/" + ns_ + "/mapper_node/rrg";
    ros::ServiceClient client = nh->serviceClient<mapper::RRT_RRG_PRM>(service_name);
    
    if(!client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[%s mission_node] Service ""%s"" unavailable for call.", ns_.c_str(), client.getService().c_str());
    }

    mapper::RRT_RRG_PRM path_plan_msg;
    path_plan_msg.request.max_time = 0.5;
    path_plan_msg.request.max_nodes = 4000;
    path_plan_msg.request.steer_param = 0.25;
    path_plan_msg.request.free_space_only = false;
    path_plan_msg.request.origin = origin.GetXYZ();
    path_plan_msg.request.destination = helper::eigenvec2rospoint(des_point);
    path_plan_msg.request.box_min = helper::set_rospoint(-5, -3, -0.1);
    path_plan_msg.request.box_max = helper::set_rospoint( 5,  3, 1.2);
    path_plan_msg.request.prune_result = true;
    path_plan_msg.request.publish_rviz = true;
    if (client.call(path_plan_msg)) {
        if(path_plan_msg.response.success == false) {
            ROS_ERROR("[%s mission_node] Path Planner could not find a feasible solution.", ns_.c_str());
        }
    } else {
        ROS_ERROR("[%s mission_node] Failed to call path planning service %s.",
                  ns_.c_str(), client.getService().c_str());
    }
    ROS_INFO("Planning time: %f", path_plan_msg.response.planning_time);
    ROS_INFO("Planning nodes: %d", path_plan_msg.response.n_nodes);

    std::vector<mission_planner::xyz_heading> waypoint_list;
    for (uint i = 0; i < path_plan_msg.response.path.size(); i++) {
        Eigen::Vector3d pt = helper::rospoint2eigenvec(path_plan_msg.response.path[i]);
        std::cout << pt.transpose() << std::endl;
        waypoint_list.push_back(mission_planner::xyz_heading(pt, origin.yaw_));
    }
    std::cout << std::endl;
    mission_.ReturnWhenIdle();

    // Publish inspection waypoint markers
    this->PublishWaypointMarkers(waypoint_list);

    ROS_INFO("Adding waypoints into buffer.");
    mission_.AddWaypoints2Buffer(waypoint_list, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);


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

bool InspectorClass::LoadWaypoints(const std::string &filename,
                                   std::vector<mission_planner::xyz_heading> *waypoint_list) {
    ROS_INFO("[mission_node] Opening waypoints file: \n%s\n", filename.c_str());
    std::ifstream myfile(filename.c_str());
    float x, y, z, yaw;
    float camera_to_base_link = 0.1;  // Camera is 10cm ahead of the base link in vicon
    // init_pose.getBasis().getRPY(init_roll, init_pitch, init_yaw);
    // ROS_INFO("Init yaw: %f", init_yaw);


    // Check whether file could be opened (path might be wrong)
    if (myfile.is_open()) {
        while( myfile >> x >> y >> z >> yaw) {
            const Eigen::Vector3d pos_slam(x, y, z);
            Eigen::Vector3d pos_vicon = rel_pos_slam2vicon_ + rot_slam2vicon_*pos_slam;
            pos_vicon = pos_vicon - camera_to_base_link*Eigen::Vector3d(cos(rel_yaw_slam2vicon_ + yaw), sin(rel_yaw_slam2vicon_ + yaw), 0.0);
            waypoint_list->push_back(mission_planner::xyz_heading(pos_vicon, rel_yaw_slam2vicon_ + yaw));
            // std::cout << x << " " << y << " " << z << " " << init_yaw-yaw << std::endl;
        }
        myfile.close();

        // Check if any waypoint was loaded
        if(waypoint_list->size() > 0) {
            return 1;
        } else {
            ROS_ERROR("[mission_node] No waypoints within the file: %s", filename.c_str());
            return 0;
        }
    } else {
        ROS_ERROR("[mission_node] Unable to open file in %s", filename.c_str());
        return 0;
    }
}

void InspectorClass::SetRelPoseVariables(const geometry_msgs::Pose &rel_pose) {
    rel_pos_slam2vicon_ = helper::rospoint2eigenvec(rel_pose.position);
    rot_slam2vicon_ = helper::ros2eigenquat(rel_pose.orientation).toRotationMatrix();
    Eigen::Vector3d rpy = helper::quat2rpy(rel_pose.orientation);
    rel_yaw_slam2vicon_ = rpy(2);
}

Eigen::Vector3d InspectorClass::Slam2Vicon(Eigen::Vector3d slam_point) {
    return rel_pos_slam2vicon_ + rot_slam2vicon_*slam_point;
}

Eigen::Vector3d InspectorClass::Vicon2Slam(Eigen::Vector3d vicon_point) {
    return rot_slam2vicon_.transpose()*(vicon_point - rel_pos_slam2vicon_);
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

void InspectorClass::RelTfPubTask(const geometry_msgs::Pose &pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  transform.setRotation(q);
  
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "slam"));
    loop_rate.sleep();
  }
}

bool InspectorClass::StartRelPoseEstimator(geometry_msgs::Pose *rel_pose) {
    std::string rel_pose_srv_name = "/batch_solver/start_new_batch";
    rel_pose_client_ = nh_.serviceClient<mg_msgs::RequestRelativePoseBatch>(rel_pose_srv_name);
    mg_msgs::RequestRelativePoseBatch client_msg;
    client_msg.request.data = 50;
    ROS_INFO("Calling service '%s' for batch solution!", rel_pose_client_.getService().c_str());
    if (rel_pose_client_.call(client_msg)) {
        ROS_INFO("[mission_node] Relative pose returned successfully!");
        *rel_pose = client_msg.response.pose;
        this->SetRelPoseVariables(client_msg.response.pose);
        return true;
    } else {
        ROS_WARN("[mission_node] Error calling relative pose server. Aborting!");
        return false;
    }
}

void InspectorClass::LoadOctomap() {
    std::string load_octomap_srv_name = "/" + ns_ + "/mapper_node/load_map";
    load_octomap_client_ = nh_.serviceClient<std_srvs::Trigger>(load_octomap_srv_name);
    std_srvs::Trigger client_msg;
    ROS_INFO("Calling service '%s' for starting octomap!", load_octomap_client_.getService().c_str());
    if (load_octomap_client_.call(client_msg)) {
        ROS_INFO("[mission_node] Start Octomap returned successfully!");
    } else {
        ROS_WARN("[mission_node] Error calling Start Octomap");
        return;
    }
}

} // inspector