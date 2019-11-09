

#ifndef MISSION_CLASS_H_
#define MISSION_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Local defined libraries
#include <mission_planner/drone_structs.h>
#include <mission_planner/helper.h>
#include <mission_planner/tf_class.h>
#include <mission_planner/visualization_functions.h>

// Msg/srv defined in other packages
#include "mg_msgs/minSnapWpStamped.h"
#include "mg_msgs/minSnapWpPVAJ.h"
#include "mg_msgs/PVAJ_request.h"
#include "mapper/RRT_RRG_PRM.h"

// Msg/srv types
#include "std_srvs/Trigger.h"

// ROS Action types
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <mg_msgs/follow_PVAJS_trajectoryAction.h>

// C++ libraries
#include <vector>
#include <string>
#include <thread>
#include <fstream>
#include <math.h>

// Eigen-based libraries
#include <Eigen/Dense>

namespace mission_planner {

class MissionClass {
 public:
  MissionClass() {};
  MissionClass(const std::string &ns, const double &tf_update_rate,
               const double &max_velocity, const uint &drone_index = 0);
  ~MissionClass();

  //Methods -------------------------------------------------------
  void Initialize(const std::string &ns, const double &tf_update_rate,
                  const double &max_velocity, const uint &drone_index = 0);

  // Method for starting the quadcopter to listen to PVA messages
  void SetQuadPosMode(const std::string &ns, ros::NodeHandle *nh);

  // Method for disarming the quad (stop motors)
  void DisarmQuad(const std::string &ns, ros::NodeHandle *nh);

  // Helper function to add waypoints into the buffer
  void AddWaypoints2Buffer(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,
                           const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
                           const double &sampling_time, xyz_heading *final_waypoint);

  // Same as before, but each traectory is named (for distinguishing between them in Rviz)
  void AddWaypoints2Buffer(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,  
                                       const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
                                       const double &sampling_time, const std::string &traj_name, xyz_heading *final_waypoint);

  // Helper function for adding planned trajectories to the buffer
  void AddTrajectoryToBuffer(const mission_planner::TrajectoryActionInputs &traj_inputs);

  // Add to buffer for rviz publishing
  void AddToRvizBuffer(const std::vector<xyz_heading> &xyz_heading_array,
                       const mg_msgs::PVAJS_array &flatStates,
                       const std::string &traj_name = "");

  // Add disarm command to buffer (quad disarms after finishes all other previous commands)
  void AddDisarm2Buffer();

  // Method for taking off when on the ground. Returns final position/heading
  bool Takeoff(const std::string &ns, const double &takeoff_height, const double &sampling_time,
               const double &avg_velocity, ros::NodeHandle *nh, xyz_heading *final_xyz_yaw);

  // Method for landing from a current location
  bool Land(const std::string &ns, const double &land_height, const double &sampling_time,
            const double &avg_velocity, ros::NodeHandle *nh);

  // Method for landing from a current location (use px4_control autoland)
  bool Land(const std::string &ns, ros::NodeHandle *nh);

  // Method for going straight to a final point 
  bool GoStraight2Point(const std::string &ns, const xyz_heading &destination, const double &sampling_time,
                        const double &max_velocity, ros::NodeHandle *nh);

  // Method for executing waypoint navigation  (blocks execution of code until finished)
  bool WaypointNavigation(const std::string &ns, const std::vector<xyz_heading> waypoints,
                          const Eigen::Vector3d &init_vel, const Eigen::Vector3d &final_vel,
                          const double &max_vel, const double &max_acc, const double &sampling_time,
                          ros::NodeHandle *nh);

  // Returns current pose of the vehicle - thread safe
  tf::StampedTransform GetCurrentPose();

  // Wait until the first pose is obtained
  tf::StampedTransform WaitForFirstPose();

  // Method for getting a minimum snap trajectory between two points only
  bool MinSnapPoint2Point(const std::string &ns, const Eigen::Vector3d &init_point,
                          const Eigen::Vector3d &final_point, const double &yaw0, 
                          const double &yaw_final, const double &tf, const double &sampling_time,
                          ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates);

  // Different template for the function above
  bool MinSnapPoint2Point(const std::string &ns, const xyz_heading &init_wp, 
                          const xyz_heading &final_wp, const double &tf, 
                          const double &sampling_time, ros::NodeHandle *nh,
                          mg_msgs::PVAJS_array *flatStates);

  // Method for getting minimum snap trajectories for multiple waypoints
  bool MinSnapWaypointSet(const std::string &ns, const std::vector<xyz_heading> waypoints,
                          const Eigen::Vector3d &init_vel, const Eigen::Vector3d &final_vel,
                          const double &max_vel, const double &max_acc, const double &sampling_time,
                          ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates);
  // Different template for the function above
  bool MinSnapWaypointSet(const std::string &ns, const minSnapWpInputs &Inputs,
                          ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates);

  // Calls different action types (Halt, disarm, PVAJS trajectory)
  void CallActionType(const std::string &ns, const TrajectoryActionInputs &traj_inputs, 
                      const bool &wait_until_done, ros::NodeHandle *nh,
                      actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> *client);

  // Method for sending a PVAJS trajectory to the action server
  bool CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                       const double &sampling_time, const bool &wait_until_done,
                       ros::NodeHandle *nh);

  // Similar to construction above, but the client is declared outside the function
  bool CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                       const double &sampling_time, const bool &wait_until_done,
                       ros::NodeHandle *nh,
                       actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> *client);

  // Returns whether quad is idle (not performing any actions)
  bool IsQuadIdle();

  // Blocking function that returns when quad is idle
  void ReturnWhenIdle();

  // Callbacks (see callbacks.cpp for implementation) ------------------------------
  void ActionGoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

 protected:
  // Threads for constantly updating the tfTree values
  void TfTask();

  // Thread that reads waypoints from a buffer to solve minimum snap problems
  void MinSnapSolverTask(const std::string &ns);

  // Consumer thread
  // Consumes minimum snap trajectories (sends to action server)
  void TrajectoryActionCaller(const std::string &ns);

  // Thread for publishing trajectories into ROS
  void RvizPubThread(const std::string &ns);


 private:
  // Declare global variables (structures defined in structs.h)
  globalVariables globals_;  // These variables are all mutex-protected
  mutexStruct mutexes_;

  // Namespace of the current node
  std::string ns_;

  // Thread variables
  std::thread h_tf_thread_, h_min_snap_thread_, h_trajectory_caller_thread_, h_rviz_pub_thread_;

  // Subscriber variables
  ros::Subscriber action_status_sub_;

  // Thread rates (hz)
  double tf_update_rate_;

  // Navigation parameters
  double max_velocity_;

  // Rviz trajectory color
  std_msgs::ColorRGBA traj_color_;

};

}  // namespace mission_planner

#endif  // MISSION_CLASS_H_