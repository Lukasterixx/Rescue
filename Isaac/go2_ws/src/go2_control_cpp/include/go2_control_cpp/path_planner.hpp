#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <string>
#include <unordered_map>  // groups map
#include <numeric>        // std::iota (DSU)
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <rclcpp/rclcpp.hpp>
#include "go2_control_cpp/initial_alignment.hpp"
#include "go2_control_cpp/common_types.hpp"

namespace go2_control_cpp
{

class PathPlanner : public BT::ActionNodeBase
{
public:
    PathPlanner(const std::string& name, const BT::NodeConfiguration& config);

    // alias for convenience
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;

    BT::NodeStatus tick() override;
    void halt() override;

    static BT::PortsList providedPorts();

private:

    std::vector<Panel> generatePanels(
        const std::vector<std::vector<cv::Point>>& contours,
        bool panels_face_left,
        double resolution);
    
    // Kept for backward compatibility, though main logic uses MST now
    static std::vector<Waypoint> makePanelWaypoints(
        const Panel& panel,
        const Panel* next,   // pointer to the panel in front, or nullptr if none
        int           num_wps,
        double&       dir);

    // REMOVED: drawArrow is no longer defined in the cpp (replaced by cv::arrowedLine)

    /**
     * Publish waypoints as a PoseArray on /waypoints.
     * Note: rot_angle_deg and image_center are now ignored (logic removed in cpp), 
     * but arguments kept to maintain signature compatibility.
     */
    void publishWaypoints(
      const std::vector<Waypoint>& waypoints,
      double                       resolution,
      double                       ox,
      double                       oy,
      int                          image_rows,
      const std::string&          frame_id
    );

    bool panels_face_left_;

    // action‐client for `/navigate_through_poses`
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigate_poses_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_pub_;
    go2_control_cpp::Alignment align;
    bool have_align = false;
    rclcpp::Time      align_deadline_;
    bool              align_waiting_ = false;
    std::string maps_dir_;
    std::string yaml_path_;
    std::string pgm_path_;

    // --- Unknown->Free filter params ---
    bool clear_unknown_as_free_{true};
    int  unknown_value_{205};       // typical ROS map_server unknown in PGM
    int  unknown_tolerance_{5};     // +/- window around unknown_value
};

}  // namespace go2_control_cpp


#endif // PATH_PLANNER_HPP