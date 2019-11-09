
#ifndef MISSION_HELPER_H_
#define MISSION_HELPER_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "mg_msgs/PVAJ_request.h"
#include "mg_msgs/PVA_request.h"
#include "mg_msgs/PVAJS_array.h"
#include "mg_msgs/PVAJS.h"
#include "p4_ros/PVA.h"

#include <cmath>

namespace helper {

double deg2rad(const double &deg);

double rad2deg(const double &rad);

geometry_msgs::Point set_rospoint(const double &x, const double &y, const double &z);

geometry_msgs::Point rosvec2rospoint(const geometry_msgs::Vector3 &vec);

geometry_msgs::Point rostfvec2rospoint(const tf::Vector3 &vec);

geometry_msgs::Point eigenvec2rospoint(const Eigen::Vector3d &vec);

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec);

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec, const double height);

geometry_msgs::Vector3 setvector3(const double &x, const double &y, const double &z);

geometry_msgs::Vector3 eigenvec2rosvec(const Eigen::Vector3d &vec);

geometry_msgs::Vector3 eigenvec2rosvec(const Eigen::Vector2d &vec);

Eigen::Vector3d rostfvec2eigenvec(const tf::Vector3 &vec);

Eigen::Vector3d rospoint2eigenvec(const geometry_msgs::Point & p);

Eigen::Quaterniond ros2eigenquat(const geometry_msgs::Quaternion & q);

geometry_msgs::Quaternion set_quat(const double &roll, const double &pitch, const double &yaw);

geometry_msgs::Quaternion set_quat(const double &w, const double &x, const double &y, const double &z);

Eigen::Vector3d quat2rpy(geometry_msgs::Quaternion quat);

double getHeadingFromQuat(geometry_msgs::Quaternion quat);

double getHeadingFromTransform(tf::StampedTransform transform);

mg_msgs::PVAJ_request get_empty_PVAJ();

mg_msgs::PVA_request get_empty_PVA();

std::vector<std::pair<uint, uint>> split_waypoints(const uint &n_waypoints,
	                                               const uint &wp_per_segment);

mg_msgs::PVAJS_array min_time_to_flat_states(const std::vector<p4_ros::PVA> &pva,
	                                         const double &yaw);

}  // namespace helper

#endif  // MISSION_HELPER_H_