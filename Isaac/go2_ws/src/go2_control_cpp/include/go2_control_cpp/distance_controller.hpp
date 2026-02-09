#pragma once

#include <behaviortree_cpp_v3/control_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <std_msgs/msg/empty.hpp> // <<< Changed from action_msgs

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>

namespace go2_control_cpp
{

class DistanceTickController : public BT::ControlNode
{
public:
  DistanceTickController(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ControlNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // TF Setup
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber Setup for Retick
    // Default topic name is /retick, but can be overridden via port
    std::string retick_topic = "/retick";
    auto port_it = config.input_ports.find("retick_topic");
    if (port_it != config.input_ports.end()) {
        retick_topic = port_it->second;
    }

    RCLCPP_INFO(node_->get_logger(), 
        "[DistanceTickController] Subscribing to manual retick trigger: %s", retick_topic.c_str());

    // Subscription to Empty message
    retick_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        retick_topic, 10,
        std::bind(&DistanceTickController::retickCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("dist_m", 0.5, "Distance interval (meters) to trigger children"),
      BT::InputPort<std::string>("global_frame", "odom", "Global frame"),
      BT::InputPort<std::string>("base_frame",   "livox_frame", "Base frame"),
      BT::InputPort<bool>("warmup_require_success", true, "Warm-up config"),
      BT::InputPort<std::string>("retick_topic", "/retick", "Topic to force a retick")
    };
  }

  void retickCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
      // Thread-safe set
      std::lock_guard<std::mutex> lock(retick_mutex_);
      retick_requested_ = true;
      RCLCPP_INFO(node_->get_logger(), "[DistanceTickController] Retick requested via topic!");
  }

  BT::NodeStatus tick() override
  {
    // 1. Read Ports
    const double      dist_m        = getInputOr<double>("dist_m", 0.5);
    const std::string global_frame  = getInputOr<std::string>("global_frame", "odom");
    const std::string base_frame    = getInputOr<std::string>("base_frame", "livox_frame");
    const bool        require_succ  = getInputOr<bool>("warmup_require_success", true);

    if (!warmup_inited_ && children_nodes_.size() > 0) {
      warmup_done_.assign(children_nodes_.size(), false);
      warmup_inited_ = true;
    }

    // 2. Accumulate Distance
    try {
      geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(global_frame, base_frame, tf2::TimePointZero);
      double x = tf.transform.translation.x;
      double y = tf.transform.translation.y;

      if (!last_xy_) {
        last_xy_.emplace(x, y);
      } else {
        accum_m_ += std::hypot(x - last_xy_->first, y - last_xy_->second);
        last_xy_  = std::make_pair(x, y);
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(node_->get_logger(), "DistanceTickController TF: %s", ex.what());
    }

    // 3. Check for External Retick Request
    bool force_trigger = false;
    {
        std::lock_guard<std::mutex> lock(retick_mutex_);
        if (retick_requested_) {
            force_trigger = true;
            retick_requested_ = false; // consume the event
        }
    }
    
    // If external trigger received, force accumulator to threshold
    if (force_trigger && mode_ == Mode::NORMAL) {
        RCLCPP_INFO(node_->get_logger(), "[DistanceTickController] Force triggering due to /retick");
        accum_m_ = dist_m + 0.1; 
    }

    // 4. Handle Async Children 
    if (active_child_) {
      BT::TreeNode* child = children_nodes_[*active_child_];
      const BT::NodeStatus st = child->executeTick();

      if (st != BT::NodeStatus::RUNNING) {
        child->halt();
        if (mode_ == Mode::WARM_UP) {
          warmup_done_[*active_child_] = (st == BT::NodeStatus::SUCCESS) || (!require_succ && st == BT::NodeStatus::FAILURE);
        }
        active_child_.reset();
      } else {
        return (mode_ == Mode::WARM_UP) ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
      }
    }

    // 5. Phase 1: WARM-UP
    if (mode_ == Mode::WARM_UP && warmup_inited_) {
      bool all_done = true;
      for (size_t i = 0; i < children_nodes_.size(); ++i) {
        if (warmup_done_[i]) continue;
        all_done = false;

        BT::TreeNode* child = children_nodes_[i];
        const BT::NodeStatus st = child->executeTick();

        if (st == BT::NodeStatus::RUNNING) {
          active_child_ = i;
          return BT::NodeStatus::RUNNING;
        }

        if (st == BT::NodeStatus::SUCCESS || (!require_succ && st == BT::NodeStatus::FAILURE)) {
            warmup_done_[i] = true;
        } else if (st == BT::NodeStatus::FAILURE && require_succ) {
            child->halt(); 
        }
      }

      if (!all_done) return BT::NodeStatus::RUNNING;

      mode_ = Mode::NORMAL;
      for (auto* c : children_nodes_) { c->halt(); }
      accum_m_ = 0.0; 
      
      // Clear retick flag to prevent instant trigger after warm-up
      {
          std::lock_guard<std::mutex> lock(retick_mutex_);
          retick_requested_ = false;
      }
      return BT::NodeStatus::SUCCESS;
    }

    // 6. Phase 2: NORMAL
    if (mode_ == Mode::NORMAL) {
        // Trigger if Distance met OR Retick requested
        if (accum_m_ + 1e-9 < dist_m) {
            return BT::NodeStatus::SUCCESS;
        }

        // --- Trigger! ---
        accum_m_ = 0.0;

        for (size_t i = 0; i < children_nodes_.size(); ++i) {
            BT::TreeNode* child = children_nodes_[i];
            child->halt(); 

            RCLCPP_INFO(node_->get_logger(), "[DistanceTickController] Triggering child '%s'", child->name().c_str());

            const BT::NodeStatus st = child->executeTick();

            if (st == BT::NodeStatus::RUNNING) {
                active_child_ = i;
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    if (active_child_) {
      children_nodes_[*active_child_]->halt();
      active_child_.reset();
    }
    for (auto* c : children_nodes_) { c->halt(); }
    ControlNode::halt();
  }

private:
  template <typename T>
  T getInputOr(const std::string& key, const T& def) const
  {
    if (auto v = this->template getInput<T>(key)) return v.value();
    return def;
  }

  enum class Mode { WARM_UP, NORMAL };

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Retick Trigger
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr retick_sub_;
  std::mutex retick_mutex_;
  bool retick_requested_{false};

  std::optional<std::pair<double,double>> last_xy_{};
  double accum_m_{0.0};
  std::optional<size_t> active_child_{std::nullopt};

  bool warmup_inited_{false};
  Mode mode_{Mode::WARM_UP};
  std::vector<bool> warmup_done_;
};

} // namespace go2_control_cpp