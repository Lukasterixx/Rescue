#include "go2_control_cpp/topological_mapper.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <limits> 

namespace go2_control_cpp
{

TopologicalMapperAction::TopologicalMapperAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      node_(rclcpp::Node::make_shared("topological_mapper_node")),
      last_attempt_time_(node_->get_clock()->now()),
      last_room_idx_(-1)
{
    sub_map_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map2d", 10, std::bind(&TopologicalMapperAction::mapCallback, this, std::placeholders::_1));

    pub_markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/topological_map_markers", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    exec_ = std::thread([this]() { executor_->spin(); });
}

TopologicalMapperAction::~TopologicalMapperAction()
{
    if (executor_) executor_->cancel();
    if (exec_.joinable()) exec_.join();
}

BT::PortsList TopologicalMapperAction::providedPorts()
{
    return { 
        BT::InputPort<std::string>("robot_frame", "livox_frame", "Frame attached to robot"),
        BT::InputPort<std::string>("global_frame", "odom", "Global frame (origin)")
    };
}

BT::NodeStatus TopologicalMapperAction::onStart()
{
    getInput("robot_frame", robot_frame_);
    getInput("global_frame", global_frame_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TopologicalMapperAction::onRunning()
{
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!latest_map_) return BT::NodeStatus::RUNNING; 
    }

    double rx = 0.0, ry = 0.0;
    try {
        auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        rx = t.transform.translation.x;
        ry = t.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
        return BT::NodeStatus::RUNNING;
    }

    int current_room = getRobotRoomIndex(rx, ry);

    if (current_room != -1) {
        last_room_idx_ = current_room;
    } 
    else {
        bool safe_to_map = false;
        double nearest_door_len = 0.0;

        if (all_doorways_.empty()) {
            safe_to_map = true; 
        } else {
            double min_dist = std::numeric_limits<double>::max();

            for (const auto& d : all_doorways_) {
                cv::Point2f center((d.p1.x + d.p2.x) / 2.0, (d.p1.y + d.p2.y) / 2.0);
                double dist = cv::norm(cv::Point2f(rx, ry) - center);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_door_len = cv::norm(d.p1 - d.p2);
                }
            }

            double required_dist = 0.5 * nearest_door_len;
            if (min_dist >= required_dist) {
                safe_to_map = true;
            }
        }

        if (safe_to_map) {
            auto now = node_->get_clock()->now();
            if ((now - last_attempt_time_).seconds() > 2.0) {
                
                bool room_regenerated = false;
                
                if (last_room_idx_ != -1 && last_room_idx_ < (int)discovered_rooms_.size()) {
                    room_regenerated = tryRegenerateRoom(last_room_idx_, nearest_door_len, rx, ry);
                }

                if (!room_regenerated) {
                    RCLCPP_INFO(node_->get_logger(), "Robot cleared doorway threshold. Initiating new room mapping...");
                    discoverNewRoom(rx, ry); // Uses standard default box
                }
                
                publishVisualizations();
                last_attempt_time_ = now;
                last_room_idx_ = -1; 
            }
        }
    }

    return BT::NodeStatus::RUNNING;
}

void TopologicalMapperAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Topological Mapper Halted.");
}

void TopologicalMapperAction::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = msg;
    resolution_ = msg->info.resolution;
}

int TopologicalMapperAction::getRobotRoomIndex(double rx, double ry)
{
    cv::Point2f robot_pt(rx, ry);
    for (size_t i = 0; i < discovered_rooms_.size(); ++i) {
        if (cv::pointPolygonTest(discovered_rooms_[i].corners, robot_pt, false) >= 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

cv::Mat TopologicalMapperAction::gridToMat(const nav_msgs::msg::OccupancyGrid& grid)
{
    cv::Mat mat(grid.info.height, grid.info.width, CV_8UC1);
    for (size_t i = 0; i < grid.data.size(); ++i) {
        int val = grid.data[i];
        if (val == -1) mat.data[i] = 205; 
        else if (val > 50) mat.data[i] = 0; 
        else mat.data[i] = 255; 
    }
    return mat;
}

bool TopologicalMapperAction::tryRegenerateRoom(int room_idx, double door_len, double rx, double ry)
{
    (void)rx; 
    (void)ry;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_copy = latest_map_;
    }

    cv::Mat img = gridToMat(*map_copy);
    double res = map_copy->info.resolution;
    double origin_x = map_copy->info.origin.position.x;
    double origin_y = map_copy->info.origin.position.y;

    cv::Mat working_img = img.clone();
    int line_thickness = std::max(1, static_cast<int>(0.05 / res)); 

    for (size_t i = 0; i < discovered_rooms_.size(); ++i) {
        if ((int)i == room_idx) continue; 
        std::vector<cv::Point> pts;
        for (const auto& corner : discovered_rooms_[i].corners) {
            pts.push_back(cv::Point(std::round((corner.x - origin_x) / res),
                                    std::round((corner.y - origin_y) / res)));
        }
        cv::polylines(working_img, pts, true, cv::Scalar(0), line_thickness);
    }

    auto& old_room = discovered_rooms_[room_idx];
    for (const auto& d : all_doorways_) {
        bool is_old_door = false;
        for (const auto& rd : old_room.doors) {
            if (cv::norm(d.p1 - rd.p1) < 0.01 && cv::norm(d.p2 - rd.p2) < 0.01) {
                is_old_door = true; break;
            }
        }
        if (!is_old_door) {
            cv::Point pt1(std::round((d.p1.x - origin_x) / res), std::round((d.p1.y - origin_y) / res));
            cv::Point pt2(std::round((d.p2.x - origin_x) / res), std::round((d.p2.y - origin_y) / res));
            cv::line(working_img, pt1, pt2, cv::Scalar(0), line_thickness);
        }
    }

    int old_left = std::round((old_room.corners[0].x - origin_x) / res);
    int old_top = std::round((old_room.corners[0].y - origin_y) / res);
    int old_right = std::round((old_room.corners[2].x - origin_x) / res);
    int old_bottom = std::round((old_room.corners[2].y - origin_y) / res);

    int left = old_left, right = old_right, top = old_top, bottom = old_bottom;

    bool expand_left = true, expand_right = true, expand_top = true, expand_bottom = true;
    double stop_ratio = 0.20;
    int obstacle_thresh = 250;
    int max_room_dim = 25.0 / res;

    while (expand_left || expand_right || expand_top || expand_bottom) {
        if (expand_left && left > 0) {
            cv::Mat col = working_img(cv::Range(top, bottom + 1), cv::Range(left - 1, left));
            if ((double)cv::countNonZero(col < obstacle_thresh) / col.rows > stop_ratio) expand_left = false;
            else left--;
        } else expand_left = false;

        if (expand_right && right < img.cols - 1) {
            cv::Mat col = working_img(cv::Range(top, bottom + 1), cv::Range(right + 1, right + 2));
            if ((double)cv::countNonZero(col < obstacle_thresh) / col.rows > stop_ratio) expand_right = false;
            else right++;
        } else expand_right = false;

        if (expand_top && top > 0) {
            cv::Mat row = working_img(cv::Range(top - 1, top), cv::Range(left, right + 1));
            if ((double)cv::countNonZero(row < obstacle_thresh) / row.cols > stop_ratio) expand_top = false;
            else top--;
        } else expand_top = false;

        if (expand_bottom && bottom < img.rows - 1) {
            cv::Mat row = working_img(cv::Range(bottom + 1, bottom + 2), cv::Range(left, right + 1));
            if ((double)cv::countNonZero(row < obstacle_thresh) / row.cols > stop_ratio) expand_bottom = false;
            else bottom++;
        } else expand_bottom = false;
    }

    if ((right - left) > max_room_dim || (bottom - top) > max_room_dim) return false; 

    int min_expand_px = std::max(1, static_cast<int>(std::round((0.5 * door_len) / res)));

    bool expanded_significantly = ((old_left - left >= min_expand_px) || (right - old_right >= min_expand_px) ||
                                   (old_top - top >= min_expand_px) || (bottom - old_bottom >= min_expand_px));

    int brush_outside = 4, brush_inside = 3;
    int min_door_width_px = 0.5 / res;
    int new_door_count = 0;

    int t_start = std::max(0, top - brush_outside);
    int t_end = std::min(img.rows, top + brush_inside);
    if (t_end > t_start) new_door_count += extractDoorwaysBrush(img(cv::Range(t_start, t_end), cv::Range(left, right + 1)), 0, min_door_width_px).size();

    int b_start = std::max(0, bottom - brush_inside + 1);
    int b_end = std::min(img.rows, bottom + 1 + brush_outside);
    if (b_end > b_start) new_door_count += extractDoorwaysBrush(img(cv::Range(b_start, b_end), cv::Range(left, right + 1)), 0, min_door_width_px).size();

    int l_start = std::max(0, left - brush_outside);
    int l_end = std::min(img.cols, left + brush_inside);
    if (l_end > l_start) new_door_count += extractDoorwaysBrush(img(cv::Range(top, bottom + 1), cv::Range(l_start, l_end)), 1, min_door_width_px).size();

    int r_start = std::max(0, right - brush_inside + 1);
    int r_end = std::min(img.cols, right + 1 + brush_outside);
    if (r_end > r_start) new_door_count += extractDoorwaysBrush(img(cv::Range(top, bottom + 1), cv::Range(r_start, r_end)), 1, min_door_width_px).size();

    bool doorways_changed = (new_door_count != (int)old_room.doors.size());

    if (expanded_significantly || doorways_changed) {
        
        // --- NEW: Calculate the 50% bounding box to bypass internal clutter ---
        int orig_w = old_right - old_left;
        int orig_h = old_bottom - old_top;
        
        // Contract the walls inwards by 25% on each side (leaving the middle 50%)
        int init_l = old_left + orig_w / 4;
        int init_r = old_right - orig_w / 4;
        int init_t = old_top + orig_h / 4;
        int init_b = old_bottom - orig_h / 4;

        // Fallback safeguard for impossibly tiny rooms
        if (init_r <= init_l) { init_l = old_left; init_r = old_right; }
        if (init_b <= init_t) { init_t = old_top; init_b = old_bottom; }

        if (expanded_significantly) {
            RCLCPP_INFO(node_->get_logger(), "Previous room expanded significantly! Deleting old boundaries and regenerating from 50%% core...");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Exception: Doorway topology changed on minor expansion. Regenerating from 50%% core.");
        }

        // 1. Delete the old doorways from the global list
        for (const auto& rd : old_room.doors) {
            all_doorways_.erase(std::remove_if(all_doorways_.begin(), all_doorways_.end(),
                [&](const Doorway& ad) {
                    return cv::norm(ad.p1 - rd.p1) < 0.01 && cv::norm(ad.p2 - rd.p2) < 0.01;
                }), all_doorways_.end());
        }

        // 2. Erase the incomplete room
        discovered_rooms_.erase(discovered_rooms_.begin() + room_idx);

        // 3. Flood-fill a brand new room using the 50% box boundaries!
        discoverNewRoom(old_room.center.x, old_room.center.y, init_l, init_r, init_t, init_b);
        return true;
    }

    return false;
}

void TopologicalMapperAction::discoverNewRoom(double rx, double ry, int init_l, int init_r, int init_t, int init_b)
{
    nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_copy = latest_map_;
    }

    cv::Mat img = gridToMat(*map_copy);
    double res = map_copy->info.resolution;
    double origin_x = map_copy->info.origin.position.x;
    double origin_y = map_copy->info.origin.position.y;

    int cx = std::round((rx - origin_x) / res);
    int cy = std::round((ry - origin_y) / res);

    if (cx < 0 || cx >= img.cols || cy < 0 || cy >= img.rows) return;

    cv::Mat working_img = img.clone();
    int line_thickness = std::max(1, static_cast<int>(0.05 / res)); 

    for (const auto& room : discovered_rooms_) {
        std::vector<cv::Point> pts;
        for (const auto& corner : room.corners) {
            pts.push_back(cv::Point(std::round((corner.x - origin_x) / res),
                                    std::round((corner.y - origin_y) / res)));
        }
        cv::polylines(working_img, pts, true, cv::Scalar(0), line_thickness);
    }

    for (const auto& d : all_doorways_) {
        cv::Point pt1(std::round((d.p1.x - origin_x) / res), std::round((d.p1.y - origin_y) / res));
        cv::Point pt2(std::round((d.p2.x - origin_x) / res), std::round((d.p2.y - origin_y) / res));
        cv::line(working_img, pt1, pt2, cv::Scalar(0), line_thickness);
    }

    double dominant_angle = 0.0; 
    
    // --- UPDATED: Use the injected box or default to 2px box ---
    int left, right, top, bottom;
    if (init_l != -1 && init_r != -1 && init_t != -1 && init_b != -1) {
        left = std::max(0, init_l);
        right = std::min(img.cols - 1, init_r);
        top = std::max(0, init_t);
        bottom = std::min(img.rows - 1, init_b);
    } else {
        int box_size = 2;
        left = std::max(0, cx - box_size);
        right = std::min(img.cols - 1, cx + box_size);
        top = std::max(0, cy - box_size);
        bottom = std::min(img.rows - 1, cy + box_size);
    }

    bool expand_left = true, expand_right = true, expand_top = true, expand_bottom = true;
    double stop_ratio = 0.20;
    int obstacle_thresh = 250;
    int max_room_dim = 25.0 / res;

    while (expand_left || expand_right || expand_top || expand_bottom) {
        if (expand_left && left > 0) {
            cv::Mat col = working_img(cv::Range(top, bottom + 1), cv::Range(left - 1, left));
            if ((double)cv::countNonZero(col < obstacle_thresh) / col.rows > stop_ratio) expand_left = false;
            else left--;
        } else expand_left = false;

        if (expand_right && right < img.cols - 1) {
            cv::Mat col = working_img(cv::Range(top, bottom + 1), cv::Range(right + 1, right + 2));
            if ((double)cv::countNonZero(col < obstacle_thresh) / col.rows > stop_ratio) expand_right = false;
            else right++;
        } else expand_right = false;

        if (expand_top && top > 0) {
            cv::Mat row = working_img(cv::Range(top - 1, top), cv::Range(left, right + 1));
            if ((double)cv::countNonZero(row < obstacle_thresh) / row.cols > stop_ratio) expand_top = false;
            else top--;
        } else expand_top = false;

        if (expand_bottom && bottom < img.rows - 1) {
            cv::Mat row = working_img(cv::Range(bottom + 1, bottom + 2), cv::Range(left, right + 1));
            if ((double)cv::countNonZero(row < obstacle_thresh) / row.cols > stop_ratio) expand_bottom = false;
            else bottom++;
        } else expand_bottom = false;
    }

    if ((right - left) > max_room_dim || (bottom - top) > max_room_dim) {
        RCLCPP_WARN(node_->get_logger(), "Room rejected: Exceeded max dimensions. W: %dpx, H: %dpx.", (right - left), (bottom - top));
        return;
    }

    Room new_room;
    new_room.color = getRandomColor();
    new_room.angle = dominant_angle;
    
    new_room.center = cv::Point2f(origin_x + ((left + right) / 2.0) * res, 
                                  origin_y + ((top + bottom) / 2.0) * res);
    
    auto pixToWorld = [&](int px, int py) -> cv::Point2f {
        return cv::Point2f(origin_x + px * res, origin_y + py * res);
    };

    new_room.corners.push_back(pixToWorld(left, top));    
    new_room.corners.push_back(pixToWorld(right, top));   
    new_room.corners.push_back(pixToWorld(right, bottom));
    new_room.corners.push_back(pixToWorld(left, bottom)); 

    int brush_outside = 4, brush_inside = 3;
    int min_door_width_px = 0.5 / res;

    int t_start = std::max(0, top - brush_outside);
    int t_end = std::min(img.rows, top + brush_inside);
    if (t_end > t_start) {
        cv::Mat region = img(cv::Range(t_start, t_end), cv::Range(left, right + 1));
        auto segments = extractDoorwaysBrush(region, 0, min_door_width_px);
        for (auto seg : segments) {
            Doorway d;
            d.p1 = pixToWorld(left + seg.first, top);
            d.p2 = pixToWorld(left + seg.second, top);
            new_room.doors.push_back(d);
            all_doorways_.push_back(d);
        }
    }

    int b_start = std::max(0, bottom - brush_inside + 1);
    int b_end = std::min(img.rows, bottom + 1 + brush_outside);
    if (b_end > b_start) {
        cv::Mat region = img(cv::Range(b_start, b_end), cv::Range(left, right + 1));
        auto segments = extractDoorwaysBrush(region, 0, min_door_width_px);
        for (auto seg : segments) {
            Doorway d;
            d.p1 = pixToWorld(left + seg.first, bottom);
            d.p2 = pixToWorld(left + seg.second, bottom);
            new_room.doors.push_back(d);
            all_doorways_.push_back(d);
        }
    }

    int l_start = std::max(0, left - brush_outside);
    int l_end = std::min(img.cols, left + brush_inside);
    if (l_end > l_start) {
        cv::Mat region = img(cv::Range(top, bottom + 1), cv::Range(l_start, l_end));
        auto segments = extractDoorwaysBrush(region, 1, min_door_width_px);
        for (auto seg : segments) {
            Doorway d;
            d.p1 = pixToWorld(left, top + seg.first);
            d.p2 = pixToWorld(left, top + seg.second);
            new_room.doors.push_back(d);
            all_doorways_.push_back(d);
        }
    }

    int r_start = std::max(0, right - brush_inside + 1);
    int r_end = std::min(img.cols, right + 1 + brush_outside);
    if (r_end > r_start) {
        cv::Mat region = img(cv::Range(top, bottom + 1), cv::Range(r_start, r_end));
        auto segments = extractDoorwaysBrush(region, 1, min_door_width_px);
        for (auto seg : segments) {
            Doorway d;
            d.p1 = pixToWorld(right, top + seg.first);
            d.p2 = pixToWorld(right, top + seg.second);
            new_room.doors.push_back(d);
            all_doorways_.push_back(d);
        }
    }

    discovered_rooms_.push_back(new_room);
    RCLCPP_INFO(node_->get_logger(), "New room registered. Total rooms: %zu", discovered_rooms_.size());
}

std::vector<std::pair<int, int>> TopologicalMapperAction::extractDoorwaysBrush(const cv::Mat& region, int axis, int min_width)
{
    std::vector<std::pair<int, int>> segments;
    if (region.empty()) return segments;

    cv::Mat line_array;
    cv::reduce(region, line_array, axis, cv::REDUCE_MIN);
    
    std::vector<int> free_pixels;
    if (axis == 0) { 
        for(int i=0; i<line_array.cols; i++) free_pixels.push_back(line_array.at<uchar>(0, i) > 100);
    } else { 
        for(int i=0; i<line_array.rows; i++) free_pixels.push_back(line_array.at<uchar>(i, 0) > 100);
    }

    int start = -1;
    for (size_t i = 0; i < free_pixels.size(); ++i) {
        if (free_pixels[i] == 1 && start == -1) start = i;
        else if (free_pixels[i] == 0 && start != -1) {
            if ((int)i - start >= min_width) segments.push_back({start, i});
            start = -1;
        }
    }
    if (start != -1 && (static_cast<int>(free_pixels.size()) - start >= min_width)) {
         segments.push_back({start, static_cast<int>(free_pixels.size()) - 1});
    }
    return segments;
}

std_msgs::msg::ColorRGBA TopologicalMapperAction::getRandomColor()
{
    std_msgs::msg::ColorRGBA color;
    color.r = ((double) rand() / (RAND_MAX));
    color.g = ((double) rand() / (RAND_MAX));
    color.b = ((double) rand() / (RAND_MAX));
    color.a = 0.35; 
    return color;
}

void TopologicalMapperAction::publishVisualizations()
{
    visualization_msgs::msg::MarkerArray msg;
    rclcpp::Time now = node_->get_clock()->now();
    int id = 0;

    for (const auto& room : discovered_rooms_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = now;
        m.ns = "rooms";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE; 
        m.action = visualization_msgs::msg::Marker::ADD;

        double width = cv::norm(room.corners[0] - room.corners[1]);
        double height = cv::norm(room.corners[1] - room.corners[2]);

        m.pose.position.x = (room.corners[0].x + room.corners[2].x) / 2.0;
        m.pose.position.y = (room.corners[0].y + room.corners[2].y) / 2.0;
        m.pose.position.z = 0.05;
        m.scale.x = width;
        m.scale.y = height;
        m.scale.z = 0.01; 
        m.color = room.color;

        msg.markers.push_back(m);
    }

    visualization_msgs::msg::Marker doors;
    doors.header.frame_id = global_frame_;
    doors.header.stamp = now;
    doors.ns = "doorways";
    doors.id = id++;
    doors.type = visualization_msgs::msg::Marker::LINE_LIST;
    doors.action = visualization_msgs::msg::Marker::ADD;
    doors.scale.x = 0.15; 
    doors.color.r = 1.0; doors.color.g = 1.0; doors.color.b = 0.0; doors.color.a = 1.0; 

    for (const auto& d : all_doorways_) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = d.p1.x; p1.y = d.p1.y; p1.z = 0.1;
        p2.x = d.p2.x; p2.y = d.p2.y; p2.z = 0.1;
        doors.points.push_back(p1);
        doors.points.push_back(p2);
    }
    
    if (!doors.points.empty()) msg.markers.push_back(doors);
    pub_markers_->publish(msg);
}

} // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::TopologicalMapperAction>(
    "TopologicalMapperAction",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::TopologicalMapperAction>(name, config);
    });
}