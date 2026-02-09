#pragma once

#include <string>
#include <array>
#include <opencv2/core.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>

namespace go2_control_cpp
{

struct CurrentPanel
{
    // Identification
    int row_number;
    std::string scan_direction; // "left" or "right"

    // Environmental / Calculated
    double height;           // Calculated z-height
    double tilt;             // Calculated tilt in degrees
    double lateral_distance; // Distance to scan line
    double row_yaw;          // Yaw of the panel axis

    // Geometry (Calculated from input corners)
    double length;
    double width;
    cv::Point2f center;
    std::array<cv::Point2f, 4> corners;
};

struct Panel
{
    int row;
    cv::Point2f center;
    std::array<cv::Point2f, 4> corners; // top left, top right, bottom left, bottom right
    int direction; // 0 to 360 true bearing
    bool is_first;
};

struct Waypoint
{
    cv::Point2f position;
    double      direction;
};

} // namespace go2_control_cpp
