
#include <mission_planner/mission_class.h>

namespace mission_planner {

// Thread for constantly updating the tfTree values
void MissionClass::TfTask() {
    ROS_DEBUG("[mission_node] tf Thread started with rate %f: ", tf_update_rate_);
    tf_listener::TfClass obj_quad2world;
    ros::Rate loop_rate(tf_update_rate_);

    std::string vehicle_frame = ns_ + "/base_link";
    std::string map_frame = "map";
    ROS_INFO("[mission_planner]: tf task started for tf from %s to %s.",
             vehicle_frame.c_str(), map_frame.c_str());

    while (ros::ok()) {
        // Get the transforms
        obj_quad2world.GetTransform(vehicle_frame , map_frame);

        mutexes_.tf.lock();
            globals_.tf_quad2world = obj_quad2world.transform_;
        mutexes_.tf.unlock();

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting tf Thread...");
}

// Consumer/producer thread
// Consume waypoints to produce minimum snap trajectories
void MissionClass::MinSnapSolverTask(const std::string &ns) {
    ROS_DEBUG("[mission_node] MinSnapSolverTask started.");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Runs at 10hz
    std::queue<mission_planner::minSnapWpInputs> min_snap_inputs_queue;
    mission_planner::minSnapWpInputs min_snap_input;

    // Load desired average velocity
    double max_vel = max_velocity_;

    while (ros::ok()) {
        // See if there is anything in the buffer
        mutexes_.waypoint_buffer.lock();
        	min_snap_inputs_queue = globals_.min_snap_inputs;
        mutexes_.waypoint_buffer.unlock();

        if(min_snap_inputs_queue.size() > 0) {
            min_snap_input = min_snap_inputs_queue.front();
        } else {
            loop_rate.sleep();
            continue;
        }

        mission_planner::TrajectoryActionInputs traj_inputs;
        if (min_snap_input.name_.compare("Disarm") == 0) {  // Disarm command
            traj_inputs.start_immediately = false;
            traj_inputs.action_type = ActionType::Disarm;
        } else {  // Trajectory command
            // Solve minimum snap problem
            if (min_snap_input.waypoints_.size() < 2) {
                ROS_WARN("Cannot solve minimum snap: not enough waypoints!");
            } else if(min_snap_input.waypoints_.size() == 2) {
                double distance = 
                    min_snap_input.waypoints_[0].GetXYZDist(min_snap_input.waypoints_[1]);
                double tf = 2.0*distance/max_vel;
                this->MinSnapPoint2Point(ns, min_snap_input.waypoints_[0],
                              min_snap_input.waypoints_[1], tf, 
                              min_snap_input.sampling_time_, &nh, &traj_inputs.flatStates);
            } else {
                this->MinSnapWaypointSet(ns, min_snap_input, &nh, &traj_inputs.flatStates);
            }
            traj_inputs.start_immediately = false;
            traj_inputs.sampling_time = min_snap_input.sampling_time_;
            traj_inputs.action_type = ActionType::Trajectory;

            // Add to list for Rviz publishing
            mutexes_.wp_traj_buffer.lock();
                globals_.wp_traj_list.push(waypoint_and_trajectory(min_snap_input.waypoints_, traj_inputs.flatStates, min_snap_input.name_));
            mutexes_.wp_traj_buffer.unlock();
        }

        // Add to buffer of quad trajectories
        mutexes_.trajectory_buffer.lock();
            globals_.traj_inputs.push_back(traj_inputs);
        mutexes_.trajectory_buffer.unlock();

        // Remove waypoints from of min snap inputs
        mutexes_.waypoint_buffer.lock();
            globals_.min_snap_inputs.pop();
        mutexes_.waypoint_buffer.unlock();

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting MinSnapSolverTask...");
}

// Consumer thread
// Consumes minimum snap trajectories
void MissionClass::TrajectoryActionCaller(const std::string &ns) {
    ROS_DEBUG("[mission_node] TrajectoryActionCaller started.");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Runs at 10hz
    const bool wait_until_done = false;
    bool action_server_busy;
    std::list<TrajectoryActionInputs> traj_inputs;
    TrajectoryActionInputs local_traj_inputs;

    mutexes_.quad_is_busy.lock();
        globals_.quad_is_busy = false;
    mutexes_.quad_is_busy.unlock();

    // Create action handle
    std::string action_name = "/" + ns + "/follow_PVAJS_trajectory_action";
    actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> 
    		followPVAJS_action_client(action_name, true);

    while (ros::ok()) {
        // Check if there is anything in the buffer
        mutexes_.trajectory_buffer.lock();
        	traj_inputs = globals_.traj_inputs;
        mutexes_.trajectory_buffer.unlock();

        // Get current state of the action server
        actionlib::SimpleClientGoalState status = followPVAJS_action_client.getState();
        if (status.state_ == status.ACTIVE) {
            action_server_busy = true;
        } else {
            action_server_busy = false;
        }

        // Set global variable of whether server is busy or not
        mutexes_.quad_is_busy.lock();
            globals_.quad_is_busy = action_server_busy;
        mutexes_.quad_is_busy.unlock();
        // std::cout << "Server status: " << action_server_busy << std::endl;

        if(traj_inputs.size() > 0) {
        	// Retrieve first item in the buffer
            local_traj_inputs = traj_inputs.front();
        } else {
            loop_rate.sleep();
            continue;
        }

        // Send data to action server (start immediately or wait until server is no longer active)
        if (local_traj_inputs.start_immediately) {
            ROS_WARN("Start immediately triggered!");
            // Start immediately is usually triggered when unsafe behaviour is detected
            followPVAJS_action_client.cancelAllGoals();
            this->CallActionType(ns, local_traj_inputs, wait_until_done, 
                                 &nh, &followPVAJS_action_client);

        } else {
            if(!action_server_busy) {
                mutexes_.quad_is_busy.lock();
                    globals_.quad_is_busy = true;
                mutexes_.quad_is_busy.unlock();
                // Start only when server is no longer busy (also removes new trajectory from buffer)
                this->CallActionType(ns, local_traj_inputs, wait_until_done, 
                                 &nh, &followPVAJS_action_client);
                // ROS_WARN("Action called!");

                // Sleep to give time for the action to become active
                ros::Duration(1.0).sleep();
            }
        }

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting TrajectoryActionCaller...");
}

void MissionClass::RvizPubThread(const std::string &ns) {
  ROS_INFO("[mission_node] Rviz trajectory publisher thread has started!");
  ros::Rate loop_rate(2);

  // Create topics for visualization
  ros::NodeHandle nh;
  ros::Publisher pathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>
                                    ("/" + ns + "/path_markers", 1);
  ros::Publisher wpMarker_pub = nh.advertise<visualization_msgs::MarkerArray>
                                    ("/" + ns + "/waypoint_markers", 1);

  // Variables
  waypoint_and_trajectory wp_and_traj;
  bool new_traj = false;
  double distance = std::numeric_limits<double>::infinity();
  std::string frame_id = "map";
  visualization_msgs::MarkerArray TrajMarkers, WaypointMarkers, delete_markers;

  // Delete current markers
  loop_rate.sleep();
  visualization_functions::deleteMarkersTemplate(frame_id, &delete_markers);
  pathMarker_pub.publish(delete_markers);
  wpMarker_pub.publish(delete_markers);

  while (ros::ok()) {
    mutexes_.wp_traj_buffer.lock();
        if(!globals_.wp_traj_list.empty()) {
            wp_and_traj = globals_.wp_traj_list.front();
            globals_.wp_traj_list.pop();
            new_traj = true;
        } else {
            new_traj = false;
        }
    mutexes_.wp_traj_buffer.unlock();

    // m_delete_markers.lock();
    //   if(delete_markers_) {
    //     TrajMarkers.markers.clear();
    //     WaypointMarkers.markers.clear();
    //     delete_markers_ = false;
    //   }
    // m_delete_markers.unlock();

    // If new trajectory, create Rviz markers
    if (new_traj) {
      if(wp_and_traj.flatStates_.PVAJS_array.size() > 0) {
        visualization_functions::drawTrajectory(wp_and_traj.flatStates_, frame_id, wp_and_traj.traj_name_, traj_color_, &TrajMarkers);
      }
      visualization_functions::drawWaypoints(wp_and_traj.Waypoints_, frame_id, &WaypointMarkers);
    
      //Publish new markers
      pathMarker_pub.publish(TrajMarkers);
      wpMarker_pub.publish(WaypointMarkers);
    }

    loop_rate.sleep();
  }
}

}  // namespace mission_planner