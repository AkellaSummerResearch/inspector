

#include <mission_planner/rover_mission_class.h>

// ---------------------------------------------------
namespace rover_planner {

RoverMissionClass::RoverMissionClass(const std::string &ns, const double &tf_update_rate,
                                     const double &max_velocity, const uint &rover_index) {
    this->Initialize(ns, tf_update_rate, max_velocity, rover_index);
}

RoverMissionClass::~RoverMissionClass() {
    // Join all threads
    h_tf_thread_.join();
    h_min_acc_thread_.join();
    h_trajectory_caller_thread_.join();
    h_rviz_pub_thread_.join();
}

void RoverMissionClass::Initialize(const std::string &ns, const double &tf_update_rate,
                                   const double &max_velocity, const uint &rover_index) {
    ns_ = ns;
    tf_update_rate_ = tf_update_rate;
    max_velocity_ = max_velocity;
    visualization_functions::SelectColor(rover_index, &traj_color_);

    // Start threads --------------------------------------------------------------
    h_tf_thread_ = std::thread(&RoverMissionClass::TfTask, this);
    h_min_acc_thread_ = std::thread(&RoverMissionClass::MinAccSolverTask, this, ns_);
    h_trajectory_caller_thread_ = std::thread(&RoverMissionClass::TrajectoryActionCaller, this, ns_);
    h_rviz_pub_thread_ = std::thread(&RoverMissionClass::RvizPubThread, this, ns_);
}

void RoverMissionClass::AddWaypoints2Buffer(const std::vector<Eigen::Vector2d> &waypoints, const Eigen::Vector2d &init_vel,
                                       const Eigen::Vector2d &final_vel, const double &max_vel, const double &max_acc,
                                       const std::string &traj_name, Eigen::Vector2d *final_waypoint) {
    if(waypoints.size() <= 1) {
        ROS_WARN("[%s mission_node] Not enough waypoints to add!", ns_.c_str());
        return;
    }

    mutexes_.waypoint_buffer.lock();
        globals_.min_acc_inputs.push(
            rover_planner::minAccWpInputs(waypoints, init_vel, final_vel, max_vel, max_acc, traj_name));
    mutexes_.waypoint_buffer.unlock();

    *final_waypoint = waypoints[waypoints.size()-1];
}

// Get current pose of the vehicle (thread-safe)
tf::StampedTransform RoverMissionClass::GetCurrentPose() {
    tf::StampedTransform transform;
    mutexes_.tf.lock();
        transform = globals_.tf_rover2world;
    mutexes_.tf.unlock();
    return transform;
}

// Wait until the first pose is obtained
tf::StampedTransform RoverMissionClass::WaitForFirstPose() {

    tf::StampedTransform tf_initial_pose;
    ros::Rate loop_rate(10);
    do {
        loop_rate.sleep();
        mutexes_.tf.lock();
            tf_initial_pose = globals_.tf_rover2world;
            //ROS_INFO("Waiting for first pose\n");
        mutexes_.tf.unlock();
    } while (tf_initial_pose.stamp_.toSec() <= 0.0);
    ROS_INFO("Waiting for first pose\n");
    return tf_initial_pose;
}

bool RoverMissionClass::MinAccPoint2Point(const std::string &ns, const Eigen::Vector2d &init_point,
                                          const Eigen::Vector2d &final_point, const double &tf, ros::NodeHandle *nh,
                                          std::vector<mg_msgs::PolyPVA> *polyX, std::vector<mg_msgs::PolyPVA> *polyY) {

    // Set service client
    std::string service_name = "/" + ns + "/min_acc_node/minAccSolver";
    ros::ServiceClient client = nh->serviceClient<mg_msgs::minAccXYWpPVA>(service_name);
    mg_msgs::PVA_request Wp0 = helper::get_empty_PVA();
    mg_msgs::PVA_request Wp1 = helper::get_empty_PVA();
    mg_msgs::PVA_request Wp_f = helper::get_empty_PVA();

    // Set first point
    Wp0.Pos = helper::eigenvec2d2rospoint(init_point);
    Wp0.Vel = helper::setvector3(0.0, 0.0, 0.0);
    Wp0.time = 0.0;
    Wp0.use_pos = true;
    Wp0.use_vel = true;

    // An intermediate point is needed for the minimum snap solver
    Wp1.Pos = helper::eigenvec2d2rospoint(0.5*(init_point + final_point));
    Wp1.time = 0.5*tf;
    Wp1.use_pos = true;

    // Set final point
    Wp_f.Pos = helper::eigenvec2d2rospoint(final_point);
    Wp_f.Vel = helper::setvector3(0.0, 0.0, 0.0);
    Wp_f.time = tf;
    Wp_f.use_pos = true;
    Wp_f.use_vel = true;

    // Push points into waypoint structure
    std::vector<mg_msgs::PVA_request> PVA_array;
    PVA_array.push_back(Wp0);
    PVA_array.push_back(Wp1);
    PVA_array.push_back(Wp_f);

    mg_msgs::minAccXYWpPVA srv;
    srv.request.PVA_array = PVA_array;
    srv.request.max_vel = 10000;  // No constraint on those in p2p trajectories
    srv.request.max_acc = 10000;
    srv.request.max_jerk = 10000;
    if (client.call(srv)) {
        ROS_INFO("[%s mission_node] Service returned succesfully with point to point trajectory!", ns_.c_str());
    } else {
        ROS_ERROR("[%s mission_node] Failed to call service ""%s"" for point to point trajectory...", ns_.c_str(),
                  client.getService().c_str());
        return false;
    }

    *polyX = srv.response.X;
    *polyY = srv.response.Y;

    return true;
}

bool RoverMissionClass::MinAccWaypointSet(const std::string &ns, const std::vector<Eigen::Vector2d> waypoints,
                                          const Eigen::Vector2d &init_vel, const Eigen::Vector2d &final_vel,
                                          const double &max_vel, const double &max_acc, ros::NodeHandle *nh,
                                          std::vector<mg_msgs::PolyPVA> *polyX, std::vector<mg_msgs::PolyPVA> *polyY) {
    // Set service client
    std::string service_name = "/" + ns + "/min_acc_node/minAccSolver";
    ros::ServiceClient client = nh->serviceClient<mg_msgs::minAccXYWpPVA>(service_name);
    geometry_msgs::Point Pos0, Pos_mid, Pos_final;
    std::vector<mg_msgs::PVA_request> PVA_array;

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
        mg_msgs::PVA_request PVA = helper::get_empty_PVA();
        
        // Add position to all waypoints
        PVA.Pos.x = waypoints[i][0];
        PVA.Pos.y = waypoints[i][1];
        PVA.use_pos = true;
        // ROS_INFO("x: %f\ty: %f\tz: %f\t", PVA.Pos.x, PVA.Pos.y, PVA.Pos.z);

        // Add velocity and acceleration to initial and final waypoints
        if (i == 0) {
            PVA.Vel = helper::eigenvec2rosvec(init_vel);
            PVA.use_vel = true;
            PVA.Acc = helper::setvector3(0.0, 0.0, 0.0);
            PVA.use_acc = true;
        } else if (i == n_w - 1) {
            PVA.Vel = helper::eigenvec2rosvec(final_vel);
            PVA.use_vel = true;
            PVA.Acc = helper::setvector3(0.0, 0.0, 0.0);
            PVA.use_acc = true;
            total_displacement += (waypoints[i] - waypoints[i-1]).norm();
        } else {
            total_displacement += (waypoints[i] - waypoints[i-1]).norm();
        }
        cumulative_displacement[i] = total_displacement;

        PVA_array.push_back(PVA);
    }

    // Estimate final time
    const double tf = 1.5*total_displacement/max_vel;

    // Set time estimates for the PVA array
    const double n_wd = n_w;    // Same value as n_w, but double data type
    for (uint i = 0; i < n_w; i++) {
        PVA_array[i].time = tf*cumulative_displacement[i]/total_displacement;
        // ROS_INFO("time[%d]: %f\tcum_displacement: %f", int(i), PVAJ_array[i].time, cumulative_displacement[i]);
    }

    mg_msgs::minAccXYWpPVA srv;
    srv.request.PVA_array = PVA_array;
    srv.request.max_vel = max_vel;
    srv.request.max_acc = max_acc;
    srv.request.max_jerk = max_jerk;
    if (client.call(srv)) {
         //ROS_INFO("Return size: %d", int(srv.response.flatStates.PVAJS_array.size()));
        ROS_INFO("[%s mission_node] Service returned succesfully with waypoint trajectory!", ns_.c_str());
    } else {
        ROS_ERROR("[%s mission_node] Failed to call service ""%s"" for waypoint trajectory...", ns_.c_str(),
                  client.getService().c_str());
        return false;
    }

    *polyX = srv.response.X;
    *polyY = srv.response.Y;

    return true;
}

bool RoverMissionClass::MinAccWaypointSet(const std::string &ns, const rover_planner::minAccWpInputs &Inputs, ros::NodeHandle *nh,
                                          std::vector<mg_msgs::PolyPVA> *polyX, std::vector<mg_msgs::PolyPVA> *polyY) {
    this->MinAccWaypointSet(ns, Inputs.waypoints_, Inputs.init_vel_, Inputs.final_vel_,
                            Inputs.max_vel_, Inputs.max_acc_, nh, polyX, polyY);
}

void RoverMissionClass::CallActionType(const std::string &ns, const rover_planner::TrajectoryActionInputs &traj_inputs, 
                                       const bool &wait_until_done, ros::NodeHandle *nh,
                                       actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> *client) {
    
    // If halt, remove all trajectories from list and stop action
    if(traj_inputs.action_type == rover_planner::ActionType::Halt) {
        // Remove all trajectories from list
        mutexes_.trajectory_buffer.lock();
            while(globals_.traj_inputs.size() > 0) {
                globals_.traj_inputs.pop_front();
            }
        mutexes_.trajectory_buffer.unlock();
    } else if (traj_inputs.action_type == rover_planner::ActionType::Trajectory) {
        this->CallPVAAction(ns, traj_inputs, wait_until_done, nh, client);
        
        // Remove trajectory from list
        mutexes_.trajectory_buffer.lock();
            globals_.traj_inputs.pop_front();
        mutexes_.trajectory_buffer.unlock();
    }
}

bool RoverMissionClass::CallPVAAction(const std::string &ns, const rover_planner::TrajectoryActionInputs &traj_inputs,
                                      const bool &wait_until_done, ros::NodeHandle *nh) {
    // Send takeoff trajectory to px4_control action that handles trajectories
    // std::string action_name = "/" + ns + "/follow_PolyPVA_trajectory_action";
    std::string action_name = ns.c_str();
    actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> 
        followPVA_action_client(action_name, true);

    return this->CallPVAAction(ns, traj_inputs, wait_until_done, nh, &followPVA_action_client);
}

bool RoverMissionClass::CallPVAAction(const std::string &ns, const rover_planner::TrajectoryActionInputs &traj_inputs,
                                      const bool &wait_until_done, ros::NodeHandle *nh,
                                      actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> *client) {
    ros::Duration timeout(2.0);
    if(!client->waitForServer()) {
        ROS_ERROR("[%s mission_node] Failed to call action server for sending trajectory: timeout!", ns_.c_str());
        return false;
    }

    ROS_INFO("[%s mission_node] Action server started, sending goal.", ns_.c_str());
    mg_msgs::follow_PolyPVA_XY_trajectoryGoal action_msg_goal;
    action_msg_goal.X = traj_inputs.polyX;
    action_msg_goal.Y = traj_inputs.polyY;
    client->sendGoal(action_msg_goal);

    if (wait_until_done) {
        bool finished_before_timeout = client->waitForResult();
    }

    return true;
}

bool RoverMissionClass::IsRoverIdle() {
    mutexes_.waypoint_buffer.lock();
        uint wp_buffer_size = globals_.min_acc_inputs.size();
    mutexes_.waypoint_buffer.unlock();

    mutexes_.trajectory_buffer.lock();
        uint traj_buffer_size = globals_.traj_inputs.size();
    mutexes_.trajectory_buffer.unlock();

    mutexes_.rover_is_busy.lock();
        bool rover_is_busy = globals_.rover_is_busy;
    mutexes_.rover_is_busy.unlock();

    // ROS_INFO("%d\t%d\t%d", (wp_buffer_size > 0), (traj_buffer_size > 0), quad_is_busy);

    if ((wp_buffer_size > 0) || (traj_buffer_size > 0) || rover_is_busy) {
        return false;
    } else {
        return true;
    }
}


void RoverMissionClass::ReturnWhenIdle() {
    ros::Rate loop_rate(10); // Check if its idle at 10hz
    while(!this->IsRoverIdle()) {
        loop_rate.sleep();
    }
}

} // namespace rover_planner