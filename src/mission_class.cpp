

#include <mission_planner/mission_class.h>

// ---------------------------------------------------
namespace mission_planner {

MissionClass::MissionClass(const std::string &ns, const double &tf_update_rate,
                           const double &max_velocity, const uint &drone_index) {
    this->Initialize(ns, tf_update_rate, max_velocity, drone_index);
}

MissionClass::~MissionClass() {
    // Join all threads
    h_tf_thread_.join();
    h_min_snap_thread_.join();
    h_trajectory_caller_thread_.join();
    h_rviz_pub_thread_.join();
}

void MissionClass::Initialize(const std::string &ns, const double &tf_update_rate,
                              const double &max_velocity, const uint &drone_index) {
    ns_ = ns;
    tf_update_rate_ = tf_update_rate;
    max_velocity_ = max_velocity;
    visualization_functions::SelectColor(drone_index, &traj_color_);

    // Start threads --------------------------------------------------------------
    h_tf_thread_ = std::thread(&MissionClass::TfTask, this);
    h_min_snap_thread_ = std::thread(&MissionClass::MinSnapSolverTask, this, ns_);
    h_trajectory_caller_thread_ = std::thread(&MissionClass::TrajectoryActionCaller, this, ns_);
    h_rviz_pub_thread_ = std::thread(&MissionClass::RvizPubThread, this, ns_);
}

void MissionClass::SetQuadPosMode(const std::string &ns, ros::NodeHandle *nh) {
    ROS_INFO("[%s mission_node] Requesting Vehicle to Start Position Mode! This requires px4_control_node to be executing!", ns_.c_str());

    std::string service_name = "/" + ns + "/px4_control_node/setQuadPVAMode";
    ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>(service_name);
    
    if(!client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[%s mission_node] Service ""%s"" unavailable for call.", ns_.c_str(), client.getService().c_str());
    }

    std_srvs::Trigger trigger_msg;
    if (client.call(trigger_msg)) {
        if(trigger_msg.response.success == false) {
            ROS_ERROR("[%s mission_node] Failed to set vehicle to listen to position commands.", ns_.c_str());
        }
    } else {
        ROS_ERROR("[%s mission_node] Failed to call service.", ns_.c_str());
    }
}

void MissionClass::DisarmQuad(const std::string &ns, ros::NodeHandle *nh) {
    ROS_INFO("[%s mission_node] Requesting Vehicle to Disarm Motors! This requires px4_control_node to be executing!", ns_.c_str());

    std::string service_name = "/" + ns + "/px4_control_node/disarmQuad";
    ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>(service_name);
    
    if(!client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[%s mission_node] Service ""%s"" unavailable for call.", ns_.c_str(), client.getService().c_str());
    }

    std_srvs::Trigger trigger_msg;
    if (client.call(trigger_msg)) {
        if(trigger_msg.response.success == false) {
            ROS_ERROR("[%s mission_node] Failed to set vehicle to listen to disarm.", ns_.c_str());
        }
    } else {
        ROS_ERROR("[%s mission_node] Failed to call service.", ns_.c_str());
    }
}

void MissionClass::AddWaypoints2Buffer(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,
                                       const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
                                       const double &sampling_time, xyz_heading *final_waypoint) {
    if(waypoints.size() <= 1) {
        ROS_WARN("[%s mission_node] Not enough waypoints to add!", ns_.c_str());
        return;
    }

    mutexes_.waypoint_buffer.lock();
        globals_.min_snap_inputs.push(minSnapWpInputs(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, ""));
    mutexes_.waypoint_buffer.unlock();

    *final_waypoint = waypoints[waypoints.size()-1];
}

void MissionClass::AddWaypoints2Buffer(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,
                                       const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
                                       const double &sampling_time, const std::string &traj_name, xyz_heading *final_waypoint) {
    if(waypoints.size() <= 1) {
        ROS_WARN("[%s mission_node] Not enough waypoints to add!", ns_.c_str());
        return;
    }

    mutexes_.waypoint_buffer.lock();
        globals_.min_snap_inputs.push(minSnapWpInputs(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, traj_name));
    mutexes_.waypoint_buffer.unlock();

    *final_waypoint = waypoints[waypoints.size()-1];
}

void MissionClass::AddTrajectoryToBuffer(const mission_planner::TrajectoryActionInputs &traj_inputs) {

    // Add to buffer of quad trajectories
    mutexes_.trajectory_buffer.lock();
        globals_.traj_inputs.push_back(traj_inputs);
    mutexes_.trajectory_buffer.unlock();
}

void MissionClass::AddToRvizBuffer(const std::vector<xyz_heading> &xyz_heading_array,
                                   const mg_msgs::PVAJS_array &flatStates,
                                   const std::string &traj_name) {
    // Add to list for Rviz publishing
    mutexes_.wp_traj_buffer.lock();
        globals_.wp_traj_list.push(mission_planner::waypoint_and_trajectory(xyz_heading_array, flatStates, traj_name));
    mutexes_.wp_traj_buffer.unlock();
}

void MissionClass::AddDisarm2Buffer() {
    mutexes_.waypoint_buffer.lock();
        globals_.min_snap_inputs.push(minSnapWpInputs("Disarm"));
    mutexes_.waypoint_buffer.unlock();
}

// Method for taking off when on the ground 
bool MissionClass::Takeoff(const std::string &ns, const double &takeoff_height, const double &sampling_time,
                           const double &avg_velocity, ros::NodeHandle *nh, xyz_heading *final_xyz_yaw) {
    // Get current pose
    tf_listener::TfClass tf_initial_pose;
    mutexes_.tf.lock();
        tf_initial_pose.transform_ = globals_.tf_quad2world;
    mutexes_.tf.unlock();
    double yaw, pitch, roll;
    tf_initial_pose.transform_.getBasis().getRPY(roll, pitch, yaw);

    // Plan a minimum snap trajectory for taking-off (need to take at least 1sec to takeoff)
    double tf = std::max(2.0*std::fabs(takeoff_height)/avg_velocity, 1.0);
    Eigen::Vector3d init_pos = helper::rostfvec2eigenvec(tf_initial_pose.transform_.getOrigin());
    Eigen::Vector3d pos_final = init_pos + Eigen::Vector3d(0.0, 0.0, takeoff_height);

    // Get smooth trajectory for take-off
    mission_planner::TrajectoryActionInputs traj_inputs;
    if (!this->MinSnapPoint2Point(ns, init_pos, pos_final, yaw, yaw, 
                                  tf, sampling_time, nh, &traj_inputs.flatStates)) {
        return 0;
    }
    traj_inputs.start_immediately = false;
    traj_inputs.sampling_time = sampling_time;

    mutexes_.trajectory_buffer.lock();
        globals_.traj_inputs.push_back(traj_inputs);
    mutexes_.trajectory_buffer.unlock();

    *final_xyz_yaw = xyz_heading(pos_final, yaw);

    // Add to list for Rviz publishing
    std::string name = ns + "/takeoff";
    std::vector<xyz_heading> waypoints;
    waypoints.push_back(xyz_heading(init_pos, yaw));
    waypoints.push_back(xyz_heading(pos_final, yaw));
    mutexes_.wp_traj_buffer.lock();
        globals_.wp_traj_list.push(waypoint_and_trajectory(waypoints, traj_inputs.flatStates, name));
    mutexes_.wp_traj_buffer.unlock();

    return 1;
}

// Method for landing from a current location (plan it locally)
bool MissionClass::Land(const std::string &ns, const double &land_height, const double &sampling_time,
                        const double &avg_velocity, ros::NodeHandle *nh) {
    // Get current pose
    tf_listener::TfClass tf_initial_pose;
    mutexes_.tf.lock();
        tf_initial_pose.transform_ = globals_.tf_quad2world;
    mutexes_.tf.unlock();
    double yaw, pitch, roll;
    tf_initial_pose.transform_.getBasis().getRPY(roll, pitch, yaw);

    // Plan a minimum snap trajectory for taking-off
    Eigen::Vector3d init_pos = helper::rostfvec2eigenvec(tf_initial_pose.transform_.getOrigin());
    Eigen::Vector3d pos_final = init_pos;
    pos_final[2] = land_height;
    double height = fabs(init_pos[2] - land_height);
    double tf = std::max(0.5*height/avg_velocity, 1.0);

    // Get smooth trajectory for landing
    mission_planner::TrajectoryActionInputs traj_inputs;
    if (!this->MinSnapPoint2Point(ns, init_pos, pos_final, yaw, yaw, 
                                  tf, sampling_time, nh, &traj_inputs.flatStates)) {
        return 0;
    }
    traj_inputs.start_immediately = false;
    traj_inputs.sampling_time = sampling_time;

    mutexes_.trajectory_buffer.lock();
        globals_.traj_inputs.push_back(traj_inputs);
    mutexes_.trajectory_buffer.unlock();

    // Add to list for Rviz publishing
    std::string name = ns + "/land";
    std::vector<xyz_heading> waypoints;
    waypoints.push_back(xyz_heading(init_pos, yaw));
    waypoints.push_back(xyz_heading(pos_final, yaw));
    mutexes_.wp_traj_buffer.lock();
        globals_.wp_traj_list.push(waypoint_and_trajectory(waypoints, traj_inputs.flatStates, name));
    mutexes_.wp_traj_buffer.unlock();

    return 1;
}

// Method for landing from a current location (use px4_control autoland)
bool MissionClass::Land(const std::string &ns, ros::NodeHandle *nh) {
    ROS_INFO("[%s mission_node] Requesting Vehicle to Land! This requires px4_control_node to be executing!", ns_.c_str());

    std::string service_name = "/" + ns + "/px4_control_node/landQuad";
    ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>(service_name);
    
    if(!client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[%s mission_node] Service ""%s"" unavailable for call.", ns_.c_str(), client.getService().c_str());
    }

    std_srvs::Trigger trigger_msg;
    if (client.call(trigger_msg)) {
        if(trigger_msg.response.success == false) {
            ROS_ERROR("[%s mission_node] Failed to set vehicle to land.", ns_.c_str());
        }
    } else {
        ROS_ERROR("[%s mission_node] Failed to call service.", ns_.c_str());
    }
}

// Method for going straight to a final point
bool MissionClass::GoStraight2Point(const std::string &ns, const xyz_heading &destination, const double &sampling_time,
                                    const double &max_velocity, ros::NodeHandle *nh) {
    // Get current pose
    tf_listener::TfClass tf_initial_pose;
    mutexes_.tf.lock();
        tf_initial_pose.transform_ = globals_.tf_quad2world;
    mutexes_.tf.unlock();
    double yaw, pitch, roll;
    tf_initial_pose.transform_.getBasis().getRPY(roll, pitch, yaw);

    // Plan a minimum snap trajectory for going to destination
    Eigen::Vector3d init_pos = helper::rostfvec2eigenvec(tf_initial_pose.transform_.getOrigin());
    Eigen::Vector3d final_pos = destination.GetEigenXYZ();
    double yaw_final = destination.GetYaw();
    double tf = 2.0*(final_pos - init_pos).norm()/max_velocity;

    // Get smooth trajectory for take-off
    mission_planner::TrajectoryActionInputs traj_inputs;
    if (!this->MinSnapPoint2Point(ns, init_pos, final_pos, yaw, yaw_final, 
                                  tf, sampling_time, nh, &traj_inputs.flatStates)) {
        return 0;
    }
    traj_inputs.start_immediately = false;
    traj_inputs.sampling_time = sampling_time;

    mutexes_.trajectory_buffer.lock();
        globals_.traj_inputs.push_back(traj_inputs);
    mutexes_.trajectory_buffer.unlock();

    // Add to list for Rviz publishing
    std::string name = ns + "/p2p";
    std::vector<xyz_heading> waypoints;
    waypoints.push_back(xyz_heading(init_pos, yaw));
    waypoints.push_back(xyz_heading(final_pos, yaw));
    mutexes_.wp_traj_buffer.lock();
        globals_.wp_traj_list.push(waypoint_and_trajectory(waypoints, traj_inputs.flatStates, name));
    mutexes_.wp_traj_buffer.unlock();

    return 1;
}

// Method for executing waypoint navigation (blocks execution of code until finished) - DEPRECATED
bool MissionClass::WaypointNavigation(const std::string &ns, const std::vector<xyz_heading> waypoints,
                                      const Eigen::Vector3d &init_vel, const Eigen::Vector3d &final_vel,
                                      const double &max_vel, const double &max_acc, const double &sampling_time,
                                      ros::NodeHandle *nh) {
    // Get smooth trajectory for landing
    mg_msgs::PVAJS_array flatStates;
    if (!this->MinSnapWaypointSet(ns, waypoints, init_vel, final_vel, max_vel, max_acc, 
                                  sampling_time, nh, &flatStates)) {
        return 0;
    }

    bool wait_until_done = true;
    if (!this->CallPVAJSAction(ns, flatStates, sampling_time, wait_until_done, nh)) {
        return 0;
    }
}

// Get current pose of the vehicle (thread-safe)
tf::StampedTransform MissionClass::GetCurrentPose() {
    tf::StampedTransform transform;
    mutexes_.tf.lock();
        transform = globals_.tf_quad2world;
    mutexes_.tf.unlock();
    return transform;
}

// Wait until the first pose is obtained
tf::StampedTransform MissionClass::WaitForFirstPose() {
    tf::StampedTransform tf_initial_pose;
    ros::Rate loop_rate(10);
    do {
        loop_rate.sleep();
        mutexes_.tf.lock();
            tf_initial_pose = globals_.tf_quad2world;
        mutexes_.tf.unlock();
    } while (tf_initial_pose.stamp_.toSec() <= 0.0);
    return tf_initial_pose;
}

bool MissionClass::MinSnapPoint2Point(const std::string &ns, const Eigen::Vector3d &init_point, 
                                      const Eigen::Vector3d &final_point, const double &yaw0,
                                      const double &yaw_final, const double &tf, const double &sampling_time,
                                      ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates) {
    // Unwrap yaw
    double yaw_f = yaw_final;
    while (yaw0 - yaw_f > M_PI) {
      yaw_f = yaw_f + 2*M_PI;
    }
    while (yaw0 - yaw_f < -M_PI) {
      yaw_f = yaw_f - 2*M_PI;
    }

    // Set service client
    std::string service_name = "/" + ns + "/minSnap";
    ros::ServiceClient client = nh->serviceClient<mg_msgs::minSnapWpStamped>(service_name);
    geometry_msgs::PoseStamped Pos0, Pos_mid, Pos_final;

    // Set first point
    Pos0.pose.position = helper::eigenvec2rospoint(init_point);
    Pos0.pose.orientation = helper::set_quat(0.0, 0.0, yaw0);
    Pos0.header.stamp = ros::Time(0.0);

    // An intermediate point is needed for the minimum snap solver
    Pos_mid.pose.position = helper::eigenvec2rospoint(0.5*(init_point + final_point));
    Pos_mid.pose.orientation = helper::set_quat(0.0, 0.0, 0.5*(yaw0 + yaw_f));
    Pos_mid.header.stamp = ros::Time(0.5*tf);

    // Set final point
    Pos_final.pose.position = helper::eigenvec2rospoint(final_point);
    Pos_final.pose.orientation = helper::set_quat(0.0, 0.0, yaw_f);
    Pos_final.header.stamp = ros::Time(tf);

    // Push points into waypoint structure
    nav_msgs::Path Waypoints;
    Waypoints.poses.push_back(Pos0);
    Waypoints.poses.push_back(Pos_mid);
    Waypoints.poses.push_back(Pos_final);

    mg_msgs::minSnapWpStamped srv;
    srv.request.Waypoints = Waypoints;
    srv.request.dt_flat_states.data = sampling_time;
    if (client.call(srv))
    {
        // ROS_INFO("Return size: %d", int(srv.response.flatStates.PVAJS_array.size()));
        ROS_INFO("[%s mission_node] Service returned succesfully with point to point trajectory!", ns_.c_str());
    }
    else
    {
        ROS_ERROR("[%s mission_node] Failed to call service ""%s"" for point to point trajectory...", ns_.c_str(),
                  client.getService().c_str());
        return false;
    }

    *flatStates = srv.response.flatStates;

    return true;
}

bool MissionClass::MinSnapPoint2Point(const std::string &ns, const xyz_heading &init_wp, 
                                      const xyz_heading &final_wp, const double &tf, 
                                      const double &sampling_time, ros::NodeHandle *nh,
                                      mg_msgs::PVAJS_array *flatStates) {
    this->MinSnapPoint2Point(ns, init_wp.GetEigenXYZ(), final_wp.GetEigenXYZ(),
                             init_wp.GetYaw(), final_wp.GetYaw(), tf, sampling_time, nh, flatStates);
}

bool MissionClass::MinSnapWaypointSet(const std::string &ns, const std::vector<xyz_heading> waypoints,
                                      const Eigen::Vector3d &init_vel, const Eigen::Vector3d &final_vel,
                                      const double &max_vel, const double &max_acc, const double &sampling_time,
                                      ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates) {
    // Set service client
    std::string service_name = "/" + ns + "/minSnapOptTimeExtended";
    ros::ServiceClient client = nh->serviceClient<mg_msgs::minSnapWpPVAJ>(service_name);
    geometry_msgs::Point Pos0, Pos_mid, Pos_final;
    std::vector<mg_msgs::PVAJ_request> PVAJ_array;

    const double max_jerk = 1000.0; // No constraint on maximum jerk
    const uint n_w = waypoints.size();

    if(n_w <= 2) {
        ROS_ERROR("[%s mission_node] Not enough waypoints to plan a minimum snap trajectory", ns_.c_str());
    }

    // Set the waypoints into the appropriate structure
    std::vector<double> cumulative_displacement;
    double total_displacement = 0;
    cumulative_displacement.resize(n_w);
    for (uint i = 0; i < n_w; i++) {
        mg_msgs::PVAJ_request PVAJ = helper::get_empty_PVAJ();
        
        // Add position to all waypoints
        PVAJ.Pos = waypoints[i].GetXYZ();
        PVAJ.use_pos = true;
        // ROS_INFO("x: %f\ty: %f\tz: %f\t", PVAJ.Pos.x, PVAJ.Pos.y, PVAJ.Pos.z);

        // Add yaw to all waypoints
        PVAJ.yaw = waypoints[i].yaw_;
        PVAJ.use_yaw = true;
        // ROS_INFO("Yaw %d: %f", i+1, PVAJ.yaw);

        // Add velocity to initial and final waypoints
        if (i == 0) {
            PVAJ.Vel = helper::eigenvec2rosvec(init_vel);
            PVAJ.use_vel = true;
        } else if (i == n_w - 1) {
            PVAJ.Vel = helper::eigenvec2rosvec(final_vel);
            PVAJ.use_vel = true;
            total_displacement += (waypoints[i].GetEigenXYZ() - waypoints[i-1].GetEigenXYZ()).norm();
        } else {
            total_displacement += (waypoints[i].GetEigenXYZ() - waypoints[i-1].GetEigenXYZ()).norm();
        }
        cumulative_displacement[i] = total_displacement;

        PVAJ_array.push_back(PVAJ);
    }

    // Estimate final time
    const double tf = total_displacement/max_vel;

    // Set time estimates for the PVAJ array
    const double n_wd = n_w;    // Same value as n_w, but double data type
    for (uint i = 0; i < n_w; i++) {
        PVAJ_array[i].time = tf*cumulative_displacement[i]/total_displacement;
        // ROS_INFO("time[%d]: %f\tcum_displacement: %f", int(i), PVAJ_array[i].time, cumulative_displacement[i]);
    }


    mg_msgs::minSnapWpPVAJ srv;
    srv.request.PVAJ_array = PVAJ_array;
    srv.request.dt_flat_states.data = sampling_time;
    srv.request.max_vel= max_vel;
    srv.request.max_acc = max_acc;
    srv.request.max_jerk = max_jerk;
    if (client.call(srv))
    {
        ROS_INFO("Return size: %d", int(srv.response.flatStates.PVAJS_array.size()));
        ROS_INFO("[%s mission_node] Service returned succesfully with waypoint trajectory!", ns_.c_str());
    }
    else
    {
        ROS_ERROR("[%s mission_node] Failed to call service ""%s"" for waypoint trajectory...", ns_.c_str(),
                  client.getService().c_str());
        return false;
    }

    *flatStates = srv.response.flatStates;

    return true;
}

bool MissionClass::MinSnapWaypointSet(const std::string &ns, const minSnapWpInputs &Inputs,
                                      ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates) {
    this->MinSnapWaypointSet(ns, Inputs.waypoints_, Inputs.init_vel_, Inputs.final_vel_,
                             Inputs.max_vel_, Inputs.max_acc_, Inputs.sampling_time_,
                             nh, flatStates);
}

void MissionClass::CallActionType(const std::string &ns, const TrajectoryActionInputs &traj_inputs, 
                                  const bool &wait_until_done, ros::NodeHandle *nh,
                                  actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> *client) {
    // If halt, remove all trajectories from list and stop action
    if(traj_inputs.action_type == ActionType::Halt) {
        // Remove all trajectories from list
        mutexes_.trajectory_buffer.lock();
            while(globals_.traj_inputs.size() > 0) {
                globals_.traj_inputs.pop_front();
            }
        mutexes_.trajectory_buffer.unlock();
    } else if (traj_inputs.action_type == ActionType::Disarm) {
        this->DisarmQuad(ns_, nh);

        // Remove all trajectories from list
        mutexes_.trajectory_buffer.lock();
            while(globals_.traj_inputs.size() > 0) {
                globals_.traj_inputs.pop_front();
            }
        mutexes_.trajectory_buffer.unlock();
    } else if (traj_inputs.action_type == ActionType::Trajectory) {
        this->CallPVAJSAction(ns, traj_inputs.flatStates, traj_inputs.sampling_time, 
                              wait_until_done, nh, client);
        
        // Remove trajectory from list
        mutexes_.trajectory_buffer.lock();
            globals_.traj_inputs.pop_front();
        mutexes_.trajectory_buffer.unlock();
    }
}


bool MissionClass::CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                                     const double &sampling_time, const bool &wait_until_done,
                                     ros::NodeHandle *nh) {
    // Send takeoff trajectory to px4_control action that handles trajectories
    std::string action_name = "/" + ns + "/follow_PVAJS_trajectory_action";
    actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> 
        followPVAJS_action_client(action_name, true);

    return this->CallPVAJSAction(ns, flatStates, sampling_time, wait_until_done, nh,
                                 &followPVAJS_action_client);
}

bool MissionClass::CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                                   const double &sampling_time, const bool &wait_until_done,
                                   ros::NodeHandle *nh,
                                   actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> *client) {

    ros::Duration timeout(2.0);
    if(!client->waitForServer(timeout)) {
        ROS_ERROR("[%s mission_node] Failed to call action server for sending trajectory: timeout!", ns_.c_str());
        return false;
    }

    ROS_INFO("[%s mission_node] Action server started, sending goal.", ns_.c_str());
    mg_msgs::follow_PVAJS_trajectoryGoal action_msg_goal;
    action_msg_goal.samplingTime = sampling_time;
    action_msg_goal.flatStates = flatStates;
    client->sendGoal(action_msg_goal);

    if (wait_until_done) {
        bool finished_before_timeout = client->waitForResult();
    }

    return true;
}

bool MissionClass::IsQuadIdle() {
    mutexes_.waypoint_buffer.lock();
        uint wp_buffer_size = globals_.min_snap_inputs.size();
    mutexes_.waypoint_buffer.unlock();

    mutexes_.trajectory_buffer.lock();
        uint traj_buffer_size = globals_.traj_inputs.size();
    mutexes_.trajectory_buffer.unlock();

    mutexes_.quad_is_busy.lock();
        bool quad_is_busy = globals_.quad_is_busy;
    mutexes_.quad_is_busy.unlock();

    // ROS_INFO("%d\t%d\t%d", wp_buffer_size, traj_buffer_size, quad_is_busy);

    if ((wp_buffer_size > 0) || (traj_buffer_size > 0) || quad_is_busy) {
        return false;
    } else {
        return true;
    }
}


void MissionClass::ReturnWhenIdle() {
    ros::Rate loop_rate(10); // Check if its idle at 10hz
    while(!this->IsQuadIdle()) {
        loop_rate.sleep();
    }
}

} // namespace mission_planner