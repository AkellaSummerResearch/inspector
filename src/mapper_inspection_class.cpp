

#include <mission_planner/mapper_inspection_class.h>

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
    // nh_.getParam("max_velocity", max_velocity_);

    // Boolean for saving the octomap or not
    bool save_octomap;
    nh_.getParam("save_octomap", save_octomap);

    // // Start the Mission Planner Engine
    // ROS_INFO("[mission_node] Starting Mission Planner Engine!");    
    // mission_.Initialize(ns_, tf_update_rate, max_velocity_);

    // // Wait until measurements are available
    // ROS_INFO("[mission_node] Waiting for first pose in tf tree...");
    // tf::StampedTransform tf_initial_pose = mission_.WaitForFirstPose();
    // mission_planner::xyz_heading origin(tf_initial_pose);
    // ROS_INFO("[mission_node] First pose obtained from tf tree!");

    // Wait until relative pose finishes starting
    ros::Duration(1.0).sleep();

    // Start service for finding relative pose between SLAM and vicon frames
    geometry_msgs::Pose rel_pose;
    if (!this->StartRelPoseEstimator(&rel_pose)) {
        return;
    }

    // Create thread that publishes relative tf between vicon and slam frames
    ROS_INFO("[mission_node] Publishing relative pose between vicon and slam frames!");
    rel_tf_pub_thread_ = std::thread(&InspectorClass::RelTfPubTask, this, rel_pose);

    // Start running the octomap
    this->StartOctomap();

    // Wait for request to stop
    std::string input_string;
    ROS_INFO("Mapping has started! Press ENTER for stopping the map!");
    std::getline(std::cin, input_string);

    if (save_octomap) {
        this->SaveOctomap();
    }

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
        return true;
    } else {
        ROS_WARN("[mission_node] Error calling relative pose server. Aborting!");
        return false;
    }
}

void InspectorClass::StartOctomap() {
    std::string start_octomap_srv_name = "/" + ns_ + "/mapper_node/process_pcl";
    start_octomap_client_ = nh_.serviceClient<std_srvs::SetBool>(start_octomap_srv_name);
    std_srvs::SetBool client_msg;
    client_msg.request.data = true;
    ROS_INFO("Calling service '%s' for starting octomap!", start_octomap_client_.getService().c_str());
    if (start_octomap_client_.call(client_msg)) {
        ROS_INFO("[mission_node] Start Octomap returned successfully!");
    } else {
        ROS_WARN("[mission_node] Error calling Start Octomap");
        return;
    }
}

void InspectorClass::SaveOctomap() {
    std::string save_octomap_srv_name = "/" + ns_ + "/mapper_node/save_map";
    save_octomap_client_ = nh_.serviceClient<std_srvs::Trigger>(save_octomap_srv_name);
    std_srvs::Trigger client_msg;
    ROS_INFO("Calling service '%s' for saving octomap!", save_octomap_client_.getService().c_str());
    if (save_octomap_client_.call(client_msg)) {
        ROS_INFO("[mission_node] Save Octomap returned successfully!");
    } else {
        ROS_WARN("[mission_node] Error calling Save Octomap");
        return;
    }
}

} // inspector