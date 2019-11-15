

#ifndef COLLISION_AVOIDANCE_FIXED_MAP_CLASS_H_
#define COLLISION_AVOIDANCE_FIXED_MAP_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// Mission Planner Class
#include <mission_planner/mission_class.h>
#include <mission_planner/xyz_heading.h>

// Service types
#include <mg_msgs/RequestRelativePoseBatch.h>
#include <mg_msgs/set_strings.h>
#include <std_srvs/Trigger.h>

// Miscellaneous libraries
#include "mission_planner/helper.h"
#include "mission_planner/visualization_functions.h"

// Add message type here (testing purposes, delete later)
#include "mapper/RRT_RRG_PRM.h"

// Min time and trapezoidal planners
#include "p4_ros/min_time.h"

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

  // Method for publishing visualization markers of the desired waypoints
  void PublishWaypointMarkers(const std::vector<mission_planner::xyz_heading> &waypoint_list);

 private:

  // Namespace of the current node
  std::string ns_;

  // Navigation parameters
  double max_acceleration_, max_velocity_, max_yaw_vel_;

  // Mission class
  mission_planner::MissionClass mission_;

  // nodehandle
  ros::NodeHandle nh_;

  // Waypoint marker publisher
  ros::Publisher waypoint_marker_pub_;
  std::string markers_frame_id_ = "map";


};

}  // namespace inspector

#endif  // COLLISION_AVOIDANCE_FIXED_MAP_CLASS_H_