
#include <mission_planner/rover_mission_class.h>

namespace rover_planner {

// Thread for constantly updating the tfTree values
void RoverMissionClass::TfTask() {
    ROS_DEBUG("[mission_node] tf Thread started with rate %f: ", tf_update_rate_);
    tf_listener::TfClass obj_rover2world;
    ros::Rate loop_rate(tf_update_rate_);

    std::string vehicle_frame = ns_ + "/base_link";
    std::string map_frame = "map";

    while (ros::ok()) {
        // Get the transforms
        obj_rover2world.GetTransform(vehicle_frame , map_frame);

        mutexes_.tf.lock();
            globals_.tf_rover2world = obj_rover2world.transform_;
        mutexes_.tf.unlock();

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting tf Thread...");
}

// Consumer/producer thread
// Consume waypoints to produce smooth trajectories
void RoverMissionClass::MinAccSolverTask(const std::string &ns) {
    ROS_DEBUG("[%s mission_node] MinAccSolverTask started.", ns.c_str());

    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Runs at 10hz
    std::queue<rover_planner::minAccWpInputs> min_acc_inputs_queue;
    rover_planner::minAccWpInputs min_acc_input;

    // Load desired average velocity
    double max_vel = max_velocity_;

    while (ros::ok()) {
        // See if there is anything in the buffer
        mutexes_.waypoint_buffer.lock();
        	min_acc_inputs_queue = globals_.min_acc_inputs;
        mutexes_.waypoint_buffer.unlock();

        if(min_acc_inputs_queue.size() > 0) {
            min_acc_input = min_acc_inputs_queue.front();
        } else {
            loop_rate.sleep();
            continue;
        }

        // Solve minimum acceleration problem
        rover_planner::TrajectoryActionInputs traj_inputs;
        if (min_acc_input.waypoints_.size() < 2) {
            ROS_WARN("[%s mission_node] Cannot solve minimum acc: not enough waypoints!", ns.c_str());
        } else if(min_acc_input.waypoints_.size() == 2) {
            double distance = (min_acc_input.waypoints_[1] - min_acc_input.waypoints_[0]).norm();
            double tf = 2.0*distance/max_vel;
            this->MinAccPoint2Point(ns, min_acc_input.waypoints_[0], min_acc_input.waypoints_[1], tf, 
                                    &nh, &traj_inputs.polyX, &traj_inputs.polyY);
        } else {
            ROS_INFO("Calculating Min Acc");
            this->MinAccWaypointSet(ns, min_acc_input, &nh, &traj_inputs.polyX, &traj_inputs.polyY);
        }

        traj_inputs.start_immediately = false;
        traj_inputs.action_type = rover_planner::ActionType::Trajectory;

        // Add to list for Rviz publishing
        mutexes_.wp_traj_buffer.lock();
            globals_.wp_traj_list.push(rover_planner::waypoint_and_trajectory(
                min_acc_input.waypoints_, traj_inputs.polyX, traj_inputs.polyY, min_acc_input.name_));
        mutexes_.wp_traj_buffer.unlock();

        // Add to buffer of quad trajectories
        mutexes_.trajectory_buffer.lock();
            globals_.traj_inputs.push_back(traj_inputs);
        mutexes_.trajectory_buffer.unlock();

        // Remove waypoints from of min snap inputs
        mutexes_.waypoint_buffer.lock();
            globals_.min_acc_inputs.pop();
        mutexes_.waypoint_buffer.unlock();

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting MinSnapSolverTask...");
}

// Consumer thread
// Consumes smooth trajectories
void RoverMissionClass::TrajectoryActionCaller(const std::string &ns) {
    ROS_INFO("[mission_node] TrajectoryActionCaller started.");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Runs at 10hz
    const bool wait_until_done = false;
    bool action_server_busy;
    std::list<rover_planner::TrajectoryActionInputs> traj_inputs;
    rover_planner::TrajectoryActionInputs local_traj_inputs;

    mutexes_.rover_is_busy.lock();
        globals_.rover_is_busy = false;
    mutexes_.rover_is_busy.unlock();

    // Create action handle
    // std::string action_name = "/" + ns + "/follow_PolyPVA_trajectory_action";
    std::string action_name = "/Gollum";
    actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> 
    		followPolyPVA_action_client(action_name, true);

    while (ros::ok()) {
        // Check if there is anything in the buffer
        mutexes_.trajectory_buffer.lock();
        	traj_inputs = globals_.traj_inputs;
        mutexes_.trajectory_buffer.unlock();
        // Get current state of the action server
        actionlib::SimpleClientGoalState status = followPolyPVA_action_client.getState();
        if (status.state_ == status.ACTIVE) {
            action_server_busy = true;
        } else {
            action_server_busy = false;
        }
        // Set global variable of whether server is busy or not
        mutexes_.rover_is_busy.lock();
            globals_.rover_is_busy = action_server_busy;
        mutexes_.rover_is_busy.unlock();

        if(traj_inputs.size() > 0) {
        	// Retrieve first item in the buffer
            local_traj_inputs = traj_inputs.front();
        } else {
            loop_rate.sleep();
            continue;
        }

        // Send data to action server (start immediately or wait until server is no longer active)
        if (local_traj_inputs.start_immediately) {
            // Start immediately is usually triggered when unsafe behaviour is detected
            followPolyPVA_action_client.cancelAllGoals();
            this->CallActionType(ns, local_traj_inputs, wait_until_done, 
                                 &nh, &followPolyPVA_action_client);

        } else {
           
            if(!action_server_busy) {
                mutexes_.rover_is_busy.lock();
                    globals_.rover_is_busy = true;
                mutexes_.rover_is_busy.unlock();
                // Start only when server is no longer busy (also removes new trajectory from buffer)
                this->CallActionType(ns, local_traj_inputs, wait_until_done, 
                                 &nh, &followPolyPVA_action_client);
            }
        }

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting TrajectoryActionCaller...");
}

void RoverMissionClass::RvizPubThread(const std::string &ns) {
  ROS_INFO("[mission_node] Rviz trajectory publisher thread has started!");
  ros::Rate loop_rate(2);

  // Create topics for visualization
  ros::NodeHandle nh;
  ros::Publisher pathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>
                                    ("/" + ns + "/path_markers", 1);
  ros::Publisher wpMarker_pub = nh.advertise<visualization_msgs::MarkerArray>
                                    ("/" + ns + "/waypoint_markers", 1);

  // Variables
  rover_planner::waypoint_and_trajectory wp_and_traj;
  bool new_traj = false;
  double distance = std::numeric_limits<double>::infinity();
  std::string frame_id = "map";
  visualization_msgs::MarkerArray TrajMarkers, WaypointMarkers, delete_markers;

  // Delete current markers
  loop_rate.sleep();
  visualization_functions::deleteMarkersTemplate(frame_id, &delete_markers);
  pathMarker_pub.publish(delete_markers);
  wpMarker_pub.publish(delete_markers);

  // Trajectory variables
  const double sampling_period = 0.05; // in seconds
  const double rover_height = 0.1;    // in meters
  std::vector<Eigen::Vector2d> sampled_traj;
  std::vector<double> time;

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

    // If new trajectory, create Rviz markers
    if (new_traj) {
      wp_and_traj.trajectory_.SampleTraj(sampling_period, &sampled_traj, &time);
      if(sampled_traj.size() > 0) {
        visualization_functions::drawTrajectory(sampled_traj, rover_height, frame_id, wp_and_traj.traj_name_, traj_color_, &TrajMarkers);
      }
      visualization_functions::drawWaypoints(wp_and_traj.waypoints_, rover_height, frame_id, &WaypointMarkers);
    
      //Publish new markers
      pathMarker_pub.publish(TrajMarkers);
      wpMarker_pub.publish(WaypointMarkers);
    }

    loop_rate.sleep();
  }
}

}  // namespace rover_planner