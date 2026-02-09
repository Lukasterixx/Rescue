#include "go2_control_cpp/path_planner.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <behaviortree_cpp_v3/tree_node.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace go2_control_cpp
{

struct GraphNode {
  cv::Point2f p;      // in image pixels
  int rect_id;        // -1 for start, otherwise index of panel/rect
  int side_sign;      // +1 for the +v long side, -1 for the -v long side, 0 for start
  int side_end;       // 0 for "-u" end, 1 for "+u" end, -1 for start
};

// Return unit vectors u (along long side) and v (short-side outward normal where +v is "one long side")
inline void rectLongShortAxes(const cv::RotatedRect& r,
                              cv::Point2f& u, cv::Point2f& v,
                              float& longLen, float& shortLen)
{
  float w = r.size.width, h = r.size.height;
  longLen  = std::max(w,h);
  shortLen = std::min(w,h);

  // OpenCV's r.angle: angle of width side relative to x-axis, in [-90,0)
  float ang_deg = r.angle;
  if (w < h) ang_deg += 90.0f; // make 'ang_deg' point along the long side
  float a = ang_deg * float(CV_PI/180.0);

  u = { std::cos(a), std::sin(a) };              // along long side
  v = { -std::sin(a),  std::cos(a) };            // +90°, short-side axis (choose as "+v")
}

// build 4 outside endpoints for one rectangle
// Points are at: center ± (long/2)*u ± (short)*v
inline std::array<cv::Point2f,4> outsideLongSideEndpoints(const cv::RotatedRect& r)
{
  cv::Point2f u, v; float L, S;
  rectLongShortAxes(r, u, v, L, S);

  const cv::Point2f c = r.center;
  // Two long sides live at ±(S/2)*v. "Offset outwards by S/2" -> move another S/2
  // Net ±S * v from center to sit outside the rectangle.

  float offset_factor = 0.5f; 

  cv::Point2f offPosV = v * (S * offset_factor);   
  cv::Point2f offNegV = v * (-S * offset_factor);

  cv::Point2f along = u * (0.5f * L);

  // For each long side: endpoints at c ± along, then shifted by ±S*v
  // Indexing: [0,1] for +v side (end0 = -along, end1 = +along), [2,3] for -v side
  return {
    c - along + offPosV,  // (+v, -u end)
    c + along + offPosV,  // (+v, +u end)
    c - along + offNegV,  // (-v, -u end)
    c + along + offNegV   // (-v, +u end)
  };
}

// Preorder DFS to turn the MST into a visitation order/path
inline void dfsPreorder(int u,
                        const std::vector<std::vector<int>>& children,
                        std::vector<int>& order)
{
  order.push_back(u);
  for (int v : children[u]) dfsPreorder(v, children, order);
}

// DSU Structure for Kruskal's
struct DSU {
    std::vector<int> p, sz;
    DSU(int n): p(n), sz(n,1) { std::iota(p.begin(), p.end(), 0); }
    int  find(int x){ return p[x]==x ? x : (p[x] = find(p[x])); }
    bool unite(int a,int b){
        a = find(a); b = find(b);
        if (a == b) return false;
        if (sz[a] < sz[b]) { std::swap(a, b); }
        p[b] = a;
        sz[a] += sz[b];
        return true;
    }
};


PathPlanner::PathPlanner(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, config)
{
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string maps_dir_param;
    node->get_parameter("maps_dir", maps_dir_param);

    const std::string pkg_share = ament_index_cpp::get_package_share_directory("go2_control_cpp");

    if (!maps_dir_param.empty() && maps_dir_param[0] != '/') {
        maps_dir_param = pkg_share + "/" + maps_dir_param;
    }
    maps_dir_ = maps_dir_param; 

    yaml_path_ = maps_dir_ + "/map2d.yaml";
    pgm_path_  = maps_dir_ + "/map2d.pgm";

    navigate_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
        node, "navigate_through_poses");

    waypoint_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
        "/waypoints_farm", rclcpp::QoS{10}.transient_local());
    
    node->declare_parameter<bool>("panels_face_left");
    node->get_parameter("panels_face_left", panels_face_left_); 
    
    node->declare_parameter<bool>("clear_unknown_as_free", true);
    node->declare_parameter<int>("unknown_value", 205);
    node->declare_parameter<int>("unknown_tolerance", 5);

    node->get_parameter("clear_unknown_as_free", clear_unknown_as_free_);
    node->get_parameter("unknown_value",         unknown_value_);
    node->get_parameter("unknown_tolerance",     unknown_tolerance_);

    // --- NEW PARAMETERS FOR RESOLUTION INDEPENDENCE ---
    // Default 5.0m^2 covers small rows. 
    // Previous hardcode 500px @ 0.5res was ~125m^2 (very large).
    node->declare_parameter<double>("min_object_area_m2", 5.0); 
    // Default 1.5m helps connect lidar dots. 
    // Previous hardcode 5px @ 0.5res was 2.5m.
    node->declare_parameter<double>("morph_size_meters", 1.5); 
}

BT::PortsList PathPlanner::providedPorts()
{
    return {
        BT::InputPort<bool>("align_provided"),
        BT::InputPort<go2_control_cpp::Alignment>("alignment"),
        BT::OutputPort<double>("average_angle"),
        BT::OutputPort<std::vector<Panel>>("panels"),
        BT::OutputPort<std::vector<int>>("long_edge_indices")
    };
}

BT::NodeStatus PathPlanner::tick()
{   
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Retrieve Physical Params
    double min_object_area_m2;
    double morph_size_meters;
    node->get_parameter("min_object_area_m2", min_object_area_m2);
    node->get_parameter("morph_size_meters", morph_size_meters);

    // --- Load Map YAML ---
    YAML::Node mapCfg;
    try {
        mapCfg = YAML::LoadFile(yaml_path_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "[PathPlanner] Failed to load YAML: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }

    const std::string frame_id = "map2dglobal";
    const double resolution = mapCfg["resolution"].as<double>();
    const std::vector<double> origin = mapCfg["origin"].as<std::vector<double>>();
    const double ox = origin[0], oy = origin[1];

    // --- DYNAMIC CALCULATIONS ---
    // 1. Calculate Kernel Size (pixels) from physical size (meters)
    // Formula: Pixels = Meters / Resolution
    int kernel_dim = std::max(1, (int)std::round(morph_size_meters / resolution));
    // Ensure kernel size is odd for OpenCV
    if (kernel_dim % 2 == 0) kernel_dim++;

    // 2. Calculate Min Area (pixels) from physical area (meters^2)
    // Formula: Pixels = Meters^2 / Resolution^2
    double min_area_px = min_object_area_m2 / (resolution * resolution);

    RCLCPP_INFO(node->get_logger(), 
        "[PathPlanner] Resolution: %.3f. Morph Kernel: %dpx (%.2fm). Min Area: %.0fpx (%.2fm2)", 
        resolution, kernel_dim, morph_size_meters, min_area_px, min_object_area_m2);

    // --- Load Image ---
    cv::Mat gray = cv::imread(pgm_path_, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
        RCLCPP_ERROR(node->get_logger(), "[PathPlanner] Could not open image.");
        return BT::NodeStatus::FAILURE;
    }

    // --- Unknown->Free cleanup ---
    if (clear_unknown_as_free_) {
        const int low  = std::max(0,   unknown_value_ - unknown_tolerance_);
        const int high = std::min(255, unknown_value_ + unknown_tolerance_);
        cv::Mat unk_mask;
        cv::inRange(gray, cv::Scalar(low), cv::Scalar(high), unk_mask);
        gray.setTo(255, unk_mask);
    }

    // --- Binary Thresholding ---
    cv::Mat bw;
    cv::threshold(gray, bw, 50, 255, cv::THRESH_BINARY_INV);
    
    // --- UPDATED: Dynamic Morphology ---
    cv::morphologyEx(bw, bw, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {kernel_dim, kernel_dim}));

    cv::morphologyEx(bw, bw, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {kernel_dim, kernel_dim}));

    // --- Find Initial Contours ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // --- Filter Contours (Size/Aspect Ratio) ---
    std::vector<std::vector<cv::Point>> goodContours;
    double sumAngle = 0.0;
    int count = 0;

    for (auto& c : contours)
    {
        auto r = cv::minAreaRect(c);
        float w = r.size.width, h = r.size.height;
        float longSide = std::max(w,h);
        float shortSide = std::min(w,h);
        float area = longSide * shortSide;
        float aspect = longSide / shortSide;

        // --- UPDATED: Dynamic Area Check ---
        if (area > min_area_px && aspect > 3.0f)
        {
            goodContours.push_back(c);
            float angle = r.angle;
            if (w < h) angle += 90.0f;
            sumAngle += angle;
            ++count;
        }
    }
    
    if (goodContours.empty()) {
        RCLCPP_WARN(node->get_logger(), "[PathPlanner] Found 0 objects! Try decreasing min_object_area_m2.");
    }

    // --- Draw "Objects" Map (Unrotated) ---
    cv::Mat objects = cv::Mat::zeros(bw.size(), CV_8UC1);
    cv::drawContours(objects, goodContours, -1, cv::Scalar(255), cv::FILLED);

    double avgAngle = (count > 0) ? sumAngle/count : 0.0;
    if (avgAngle >  90.0) avgAngle -= 180.0;
    if (avgAngle <= -90.0) avgAngle += 180.0;

    // --- Start Position Calculation (Map Frame) ---
    double wx = 0.0, wy = 0.0;
    int sx = int(std::round((wx - ox) / resolution));
    int sy_map = int(std::round((wy - oy) / resolution));
    int sy = objects.rows - 1 - sy_map;
    
    sx = std::clamp(sx, 0, objects.cols-1);
    sy = std::clamp(sy, 0, objects.rows-1);

    // --- Generate Panels ---
    cv::findContours(objects, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Pass resolution to generatePanels to handle internal small-area filtering
    auto panels = generatePanels(contours, panels_face_left_, resolution);

    // ===== MST / PATH CONSTRUCTION =====

    // 1) Collect rectangles
    std::vector<cv::RotatedRect> rects;
    rects.reserve(panels.size());
    for (auto const& p : panels) {
        std::vector<cv::Point2f> pts(p.corners.begin(), p.corners.end());
        rects.push_back(cv::minAreaRect(pts));
    }

    // 2) Build Nodes
    std::vector<GraphNode> nodes;
    nodes.reserve(1 + rects.size() * 4);
    
    // Node 0: Start
    nodes.push_back(GraphNode{ cv::Point2f(float(sx), float(sy)), -1, 0, -1 });

    for (int i = 0; i < (int)rects.size(); ++i) {
        auto ep = outsideLongSideEndpoints(rects[i]);
        // Store metadata (rect_id, side_sign) to identify mandatory edges later
        nodes.push_back(GraphNode{ ep[0], i, +1, 0 });
        nodes.push_back(GraphNode{ ep[1], i, +1, 1 });
        nodes.push_back(GraphNode{ ep[2], i, -1, 0 });
        nodes.push_back(GraphNode{ ep[3], i, -1, 1 });
    }

    const int N = (int)nodes.size();

    // 3) Build Dense Edge List
    struct Edge { int u, v; float w; };
    std::vector<Edge> all_edges;
    all_edges.reserve(N * N / 2);

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            float dx = nodes[i].p.x - nodes[j].p.x;
            float dy = nodes[i].p.y - nodes[j].p.y;
            all_edges.push_back({i, j, std::sqrt(dx*dx + dy*dy)});
        }
    }

    // 4) Kruskal's with DEGREE CONSTRAINTS
    DSU dsu(N);
    std::vector<std::pair<int, int>> mst_edges;
    mst_edges.reserve(N - 1);
    
    // Track degrees to ensure nodes don't have > 2 edges (no branching)
    std::vector<int> node_degree(N, 0);

    // 4a) Force add the long sides (Mandatory)
    for (int i = 0; i < (int)rects.size(); ++i) {
        int base = 1 + 4 * i;
        // Side A
        if (dsu.unite(base + 0, base + 1)) {
            mst_edges.push_back({base + 0, base + 1});
            node_degree[base + 0]++;
            node_degree[base + 1]++;
        }
        // Side B
        if (dsu.unite(base + 2, base + 3)) {
            mst_edges.push_back({base + 2, base + 3});
            node_degree[base + 2]++;
            node_degree[base + 3]++;
        }
    }

    // 4b) Sort remaining edges and fill
    std::sort(all_edges.begin(), all_edges.end(), 
              [](const Edge& a, const Edge& b){ return a.w < b.w; });

    for (const auto& e : all_edges) {
        if ((int)mst_edges.size() == N - 1) break;

        // Condition 1: Must connect different components (No cycles)
        // Condition 2: Must not exceed degree 2 (No branching)
        if (dsu.find(e.u) != dsu.find(e.v)) {
            if (node_degree[e.u] < 2 && node_degree[e.v] < 2) {
                dsu.unite(e.u, e.v);
                mst_edges.push_back({e.u, e.v});
                node_degree[e.u]++;
                node_degree[e.v]++;
            }
        }
    }

    // 5) Build Adjacency List
    std::vector<std::vector<int>> adj(N);
    for (auto& edge : mst_edges) {
        adj[edge.first].push_back(edge.second);
        adj[edge.second].push_back(edge.first);
    }

    // 6) DFS for traversal order
    std::vector<int> parent(N, -1);
    std::vector<int> stack_dfs; 
    stack_dfs.push_back(0); 
    parent[0] = 0;

    while (!stack_dfs.empty()) {
        int u = stack_dfs.back(); stack_dfs.pop_back();
        for (int v : adj[u]) {
            if (parent[v] == -1) {
                parent[v] = u;
                stack_dfs.push_back(v);
            }
        }
    }

    std::vector<std::vector<int>> children(N);
    for (int v = 0; v < N; ++v) {
        if (v != 0 && parent[v] != -1) {
            children[parent[v]].push_back(v);
        }
    }

    std::vector<int> visitOrder;
    visitOrder.reserve(N);
    dfsPreorder(0, children, visitOrder);

    // 9) Convert to Waypoints with DIRECTION LOGIC
    std::vector<Waypoint> waypoints;
    // Parallel vector to track which GraphNode (u) created the waypoint
    std::vector<int> wp_source_indices; 
    
    waypoints.reserve(visitOrder.size());
    wp_source_indices.reserve(visitOrder.size());

    // Helper to check for Mandatory (Long) Edges
    auto isMandatory = [&](int u, int v) {
        if (u < 0 || v < 0 || u >= N || v >= N) return false;
        if (nodes[u].rect_id == -1 || nodes[v].rect_id == -1) return false;
        // Same rectangle AND same side (+v or -v)
        return (nodes[u].rect_id == nodes[v].rect_id) && 
               (nodes[u].side_sign == nodes[v].side_sign);
    };

    for (size_t k = 0; k < visitOrder.size(); ++k) {
        int u = visitOrder[k];
        double heading_deg = 0.0;
        bool angleSet = false;

        // Case A: Start of mandatory edge
        if (k + 1 < visitOrder.size()) {
            int next = visitOrder[k+1];
            if (isMandatory(u, next)) {
                float dx = nodes[next].p.x - nodes[u].p.x;
                float dy = nodes[next].p.y - nodes[u].p.y;
                heading_deg = std::atan2(-dy, dx) * 180.0 / CV_PI;
                angleSet = true;
            }
        }
        // Case B: End of mandatory edge
        if (!angleSet && k > 0) {
            int prev = visitOrder[k-1];
            if (isMandatory(prev, u)) {
                float dx = nodes[u].p.x - nodes[prev].p.x;
                float dy = nodes[u].p.y - nodes[prev].p.y;
                heading_deg = std::atan2(-dy, dx) * 180.0 / CV_PI;
                angleSet = true;
            }
        }
        // Case C: Travel Edge
        if (!angleSet) {
             if (k + 1 < visitOrder.size()) {
                 int next = visitOrder[k+1];
                 float dx = nodes[next].p.x - nodes[u].p.x;
                 float dy = nodes[next].p.y - nodes[u].p.y;
                 heading_deg = std::atan2(-dy, dx) * 180.0 / CV_PI;
             }
             else if (k > 0) {
                 int prev = visitOrder[k-1];
                 float dx = nodes[u].p.x - nodes[prev].p.x;
                 float dy = nodes[u].p.y - nodes[prev].p.y;
                 heading_deg = std::atan2(-dy, dx) * 180.0 / CV_PI;
             }
        }

        waypoints.push_back(Waypoint{{ nodes[u].p.x, nodes[u].p.y }, heading_deg});
        wp_source_indices.push_back(u); 
    }

    // 9.5) Post-process: Insert "Bulb" waypoints AND maintain indices
    std::vector<Waypoint> smoothedWaypoints;
    std::vector<int> smoothedIndices; 

    smoothedWaypoints.reserve(waypoints.size() * 1.5);
    smoothedIndices.reserve(waypoints.size() * 1.5);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        smoothedWaypoints.push_back(waypoints[i]);
        smoothedIndices.push_back(wp_source_indices[i]);

        if (i + 1 < waypoints.size()) {
            const auto& w1 = waypoints[i];
            const auto& w2 = waypoints[i+1];

            float dx_pix = w2.position.x - w1.position.x;
            float dy_pix = w2.position.y - w1.position.y;
            float dist_pix = std::sqrt(dx_pix*dx_pix + dy_pix*dy_pix);
            float dist_m = dist_pix * resolution;

            float diff = std::abs(w1.direction - w2.direction);
            if (diff > 180.0f) diff = 360.0f - diff;

            // Check Bulb Condition TURNED OFF RN
            if (false && dist_m < 5.0f && diff > 135.0f) {
                cv::Point2f mid((w1.position.x + w2.position.x) / 2.0f,
                                (w1.position.y + w2.position.y) / 2.0f);
                double w1_rad = w1.direction * CV_PI / 180.0;
                double u_img_x = std::cos(w1_rad);
                double u_img_y = -std::sin(w1_rad); // Flip Y
                double radius = dist_pix / 2.0f;
                
                cv::Point2f lateralPt;
                lateralPt.x = mid.x + u_img_x * radius;
                lateralPt.y = mid.y + u_img_y * radius;

                float chord_dx = w2.position.x - w1.position.x;
                float chord_dy = w2.position.y - w1.position.y;
                float apex_heading = std::atan2(-chord_dy, chord_dx) * 180.0 / CV_PI;

                smoothedWaypoints.push_back(Waypoint{lateralPt, apex_heading});
                smoothedIndices.push_back(-1); 
            }
        }
    }
    
    waypoints = std::move(smoothedWaypoints);
    wp_source_indices = std::move(smoothedIndices);

    // 9.6) Identify "Waypoints Remaining" counts for Long Edges
    std::vector<int> long_edge_targets; 
    int W = (int)waypoints.size();

    for (int i = 0; i < W - 1; ++i) {
        int u_idx = wp_source_indices[i];
        int v_idx = wp_source_indices[i+1];

        if (u_idx != -1 && v_idx != -1) {
            if (isMandatory(u_idx, v_idx)) {
                int target_index = i + 1;
                int remaining = W - target_index;
                long_edge_targets.push_back(remaining);
            }
        }
    }

    setOutput("long_edge_indices", long_edge_targets);

    // --- Debug Visualization ---
    {
        cv::Mat dbg;
        cv::cvtColor(objects, dbg, cv::COLOR_GRAY2BGR);

        auto drawEdge = [&](const cv::Point2f& a, const cv::Point2f& b,
                            const cv::Scalar& color, int thick = 1) {
            cv::line(dbg, a, b, color, thick, cv::LINE_AA);
        };
        auto putLbl = [&](const cv::Point2f& p, const std::string& txt,
                            const cv::Scalar& color) {
            const double fs = 0.5;
            cv::putText(dbg, txt, p + cv::Point2f(4, -4),
                        cv::FONT_HERSHEY_SIMPLEX, fs, color, 1, cv::LINE_AA);
        };

        // Draw MST Edges (Underlying structure)
        for (auto& edge : mst_edges) {
            drawEdge(nodes[edge.first].p, nodes[edge.second].p, cv::Scalar(0,255,0), 1);
        }

        // Draw Path Sequence (Thin Green Line)
        for (size_t k = 0; k + 1 < visitOrder.size(); ++k) {
            const auto& a = nodes[visitOrder[k]].p;
            const auto& b = nodes[visitOrder[k+1]].p;
            cv::line(dbg, a, b, cv::Scalar(0,255,0), 1, cv::LINE_AA);
        }

        // Labels only
        for (int i = 0; i < N; ++i) {
            putLbl(nodes[i].p, std::to_string(i), cv::Scalar(255,255,255));
        }

        const std::string dbgFile = maps_dir_ + "/mst_debug.png";
        cv::imwrite(dbgFile, dbg);
        RCLCPP_INFO(node->get_logger(), "Wrote MST debug image to %s", dbgFile.c_str());
    }
    
    // 10) Publish Waypoints
    publishWaypoints(waypoints, resolution, ox, oy, objects.rows, frame_id);

    RCLCPP_INFO(node->get_logger(), "[PathPlanner] MST Planning Complete.");

    for (const auto& p : panels) {
        // 1. Calculate angle from the stored corners
        // We find the longest edge to determine the orientation
        cv::Point2f v1 = p.corners[1] - p.corners[0];
        cv::Point2f v2 = p.corners[2] - p.corners[1];
        
        float angle_deg = 0.0f;
        // Check which side is the long side (orientation axis)
        if (cv::norm(v1) > cv::norm(v2)) {
             angle_deg = std::atan2(v1.y, v1.x) * 180.0f / CV_PI;
        } else {
             angle_deg = std::atan2(v2.y, v2.x) * 180.0f / CV_PI;
        }
        
        // Normalize angle to [-90, 90] range for readability
        // (0.0 is horizontal, negative tilts up-right, positive tilts down-right)
        while (angle_deg <= -90) angle_deg += 180;
        while (angle_deg > 90)   angle_deg -= 180;

        // 2. Format the string: "Row : Angle"
        char buf[32];
        // %.1f prints 1 decimal place (e.g., "0: 2.2")
        snprintf(buf, sizeof(buf), "%d : %.1f", p.row, angle_deg);

        // 3. Draw text
        cv::Point textOrg = p.center;
        textOrg.x -= 25; // Shift left to center the longer text
        textOrg.y += 5; 

        // Using a slightly smaller font (0.6) to fit the extra info
        cv::putText(objects, 
                    buf, 
                    textOrg, 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.6, 
                    cv::Scalar(127), // Mid-grey is visible on both white panels and black background
                    2); 
    }

    const std::string outFile = maps_dir_ + "/objects.pgm";
    cv::imwrite(outFile, objects);

    setOutput("average_angle", avgAngle);
    setOutput("panels", panels);
    
    return BT::NodeStatus::SUCCESS;
}

void PathPlanner::halt()
{
}

std::vector<Panel> PathPlanner::generatePanels(
    const std::vector<std::vector<cv::Point>>& contours,
    bool panels_face_left, 
    double resolution) // Updated signature
{
    double scan_dir = panels_face_left ? 0.0 : 180.0;
    std::vector<Panel> panels;
    panels.reserve(contours.size());

    // Calculate dynamic minimum area for panel validation (2.5m^2)
    double min_panel_px = 2.5 / (resolution * resolution);

    // 1. Create initial Panel objects from contours
    for (auto const &c : contours) {
        // --- UPDATED: Dynamic Area Check ---
        if (cv::contourArea(c) < min_panel_px) continue;
        
        auto r = cv::minAreaRect(c);
        std::array<cv::Point2f,4> pts;
        r.points(pts.data());
        
        std::sort(pts.begin(), pts.end(), [](auto &a, auto &b){ return a.y < b.y; });
        if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
        if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);

        panels.push_back(Panel{0, r.center, { pts[0], pts[1], pts[2], pts[3] }, 0, false});
    }

    if (panels.empty()) return panels;

    // 2. Sort all panels by Y first
    std::sort(panels.begin(), panels.end(), [](auto &a, auto &b){ 
        return a.center.y < b.center.y; 
    });

    // 3. Assign "Visual Row" IDs
    float rowTol = std::abs(panels[0].corners[1].y - panels[0].corners[2].y) * 0.5f;
    int visualRow = 0;
    float lastY = panels[0].center.y;
    
    panels[0].row = 0; 

    for (size_t i = 1; i < panels.size(); ++i) {
        if (std::abs(panels[i].center.y - lastY) > rowTol) {
            ++visualRow;
            lastY = panels[i].center.y;
        }
        panels[i].row = visualRow;
    }

    // 4. Sort within Visual Rows to create the "Snake" pattern
    std::vector<Panel> reordered;
    reordered.reserve(panels.size());
    int maxVisualRow = panels.back().row;

    for (int r = 0; r <= maxVisualRow; ++r) {
        std::vector<Panel> chunk;
        for (auto &p : panels) {
            if (p.row == r) chunk.push_back(p);
        }

        bool leftToRight = (r % 2 == 0);
        std::sort(chunk.begin(), chunk.end(),
                  [leftToRight](auto &a, auto &b){
                    return leftToRight ? a.center.x < b.center.x : a.center.x > b.center.x;
                  });
        
        reordered.insert(reordered.end(), chunk.begin(), chunk.end());
    }

    // 5. Final Pass: Assign UNIQUE IDs
    panels = std::move(reordered);
    for (size_t i = 0; i < panels.size(); ++i) {
        panels[i].row = static_cast<int>(i); 
        panels[i].direction = int(std::round(scan_dir)); 
        panels[i].is_first = false; 
    }
    
    if (!panels.empty()) {
        panels[0].is_first = true;
    }

    return panels;
}

void PathPlanner::publishWaypoints(
    const std::vector<Waypoint>& waypoints,
    double                       resolution,
    double                       ox,
    double                       oy,
    int                          image_rows,
    const std::string&          frame_id)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp    = rclcpp::Time(0);
  msg.header.frame_id = frame_id;
  msg.poses.reserve(waypoints.size());

  for (auto & wp : waypoints)
  {
    double wx = wp.position.x * resolution + ox;
    double wy = (image_rows - wp.position.y - 1) * resolution + oy;

    geometry_msgs::msg::Pose p;
    p.position.x = wx;
    p.position.y = wy;
    p.position.z = 0.0;

    double yaw = wp.direction * M_PI/180.0;
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);
    p.orientation = tf2::toMsg(q);

    msg.poses.push_back(p);
  }

  // --- Send to Navigation Action ---
  if (!navigate_poses_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(), "NavigateThroughPoses server not available");
    return;
  }

  NavigateThroughPoses::Goal goal_msg;
  goal_msg.behavior_tree = "";
  goal_msg.poses.reserve(msg.poses.size());
  for (auto & p : msg.poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose   = p;
      goal_msg.poses.push_back(ps);
  }

  auto send_opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions{};
  send_opts.goal_response_callback =
    [node](auto gh) {
        if (!gh) RCLCPP_ERROR(node->get_logger(), "Goal rejected");
        else     RCLCPP_INFO(node->get_logger(), "Goal accepted");
    };
  send_opts.result_callback =
    [node](const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult & r) {
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "NavigateThroughPoses succeeded");
        } else {
        RCLCPP_ERROR(node->get_logger(), "NavigateThroughPoses failed (code %d)", static_cast<int>(r.code));
        }
    };

  waypoint_pub_->publish(msg);
  navigate_poses_client_->async_send_goal(goal_msg, send_opts);
}

}  // namespace go2_control_cpp

// Register node
BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::PathPlanner>(
    "PathPlanner",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::PathPlanner>(name, config);
    });
}