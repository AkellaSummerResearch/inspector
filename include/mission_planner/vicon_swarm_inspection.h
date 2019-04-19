

#ifndef VICON_SWARM_INSPECTOR_CLASS_H_
#define VICON_SWARM_INSPECTOR_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// Mission Planner Class
#include <mission_planner/mission_class.h>
#include <mission_planner/rover_mission_class.h>
#include <mission_planner/xyz_heading.h>

// Service types
#include <mg_msgs/RequestRelativePoseBatch.h>
#include <mg_msgs/set_strings.h>
#include <std_srvs/Trigger.h>

// Miscellaneous libraries
#include "mission_planner/helper.h"
#include "mission_planner/visualization_functions.h"

// // C++ libraries
#include <vector>
#include <string>
#include <fstream>
#include <math.h>

// Eigen-based libraries
#include <Eigen/Dense>

namespace inspector {

class InspectorClass {
 public:
  InspectorClass() {};
  ~InspectorClass() {};

  // Method for executing a mission. This is implemented in different files, and then
  // the compiled one is executed
  virtual void Mission(ros::NodeHandle *nh);

 protected:

  // Method for loading waypoints from file
  bool LoadWaypoints(const std::string &filename,
                     const geometry_msgs::Pose &rel_pose,
                     std::vector<mission_planner::xyz_heading> *waypoint_list);

  // Method for publishing visualization markers of the desired waypoints
  void PublishWaypointMarkers(const std::vector<mission_planner::xyz_heading> &waypoint_list,
                              const uint &color_index);

  // Method for publishing tf transform between vicon and slam frames
  void RelTfPubTask(const geometry_msgs::Pose &pose);

 private:

  // Namespace of the current node
  std::vector<std::string> ns_;

  // List of waypoints for inspection procedure for each drone
  std::vector<std::vector<mission_planner::xyz_heading> > inspection_waypoint_list_;

  // Navigation parameters
  double max_velocity_;

  // Mission classes
  std::vector<mission_planner::MissionClass*> mission_;

  // Relative pose between vicon frame and slam frame
  geometry_msgs::Pose relative_pose_;
  ros::ServiceClient rel_pose_client_, triangulation_start_client_, triangulation_stop_client_;

  // nodehandle
  ros::NodeHandle nh_;

  // Thread for publishing transform from from vicon frame to slam frame
  std::thread rel_tf_pub_thread_;

  // Waypoint marker publisher
  ros::Publisher waypoint_marker_pub_;
  std::string markers_frame_id_ = "map";


};

}  // namespace inspector

#endif  // VICON_SWARM_INSPECTOR_CLASS_H_