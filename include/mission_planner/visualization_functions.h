/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef VISUALIZATION_FUNCTIONS_H_
#define VISUALIZATION_FUNCTIONS_H_

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <set>
#include <mission_planner/helper.h>
#include "nav_msgs/Path.h"
#include "mg_msgs/PVAJS_array.h"
#include "mg_msgs/PolyPVA.h"

namespace visualization_functions {

// Some colors for visualization markers
class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Cyan() { return Color(0.0, 1.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

void SelectColor(const uint &i, std_msgs::ColorRGBA *color);

void DrawArrowPoints(const geometry_msgs::Point& p1,
                     const geometry_msgs::Point& p2,
                     const std_msgs::ColorRGBA &color,
                     const std::string &frame_id,
                     const std::string &ns,
                     const uint &sequence_number,
                     const double &diameter,
                     visualization_msgs::Marker* marker);

void DrawArrowPoints(const Eigen::Vector3d& p1,
                     const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA &color,
                     const std::string &frame_id,
                     const std::string &ns,
                     const uint &sequence_number,
                     const double &diameter,
                     visualization_msgs::Marker* marker);

void DeleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray *marker_array);

void drawTrajectory(const mg_msgs::PVAJS_array &flatStates, 
                    const std::string &frame_id,
                    const std::string &ns,
                    const std_msgs::ColorRGBA &color,
                    visualization_msgs::MarkerArray* marker_array);

void drawTrajectory(const std::vector<Eigen::Vector2d> &trajectory,
                    const double &height,
                    const std::string &frame_id,
                    const std::string &ns,
                    const std_msgs::ColorRGBA &color,
                    visualization_msgs::MarkerArray* marker_array);

void drawWaypoints(const nav_msgs::Path &Waypoints, 
                   const std::string& frame_id,
                   visualization_msgs::MarkerArray* marker_array);

void drawWaypoints(const std::vector<Eigen::Vector2d> &Waypoints,
                   const double &height,
                   const std::string& frame_id,
                   visualization_msgs::MarkerArray* marker_array);

void deleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray* marker_array);

}  // namespace visualization_functions

#endif  // VISUALIZATION_FUNCTIONS_H_
