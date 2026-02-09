#pragma once

// --- ROS / TF / msgs ---
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>

// --- BT ---
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// --- STL / OpenCV ---
#include <atomic>
#include <thread>
#include <mutex>
#include <optional>
#include <vector>
#include <queue>
#include <string>
#include <utility>
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <array>
#include <opencv2/opencv.hpp>    // Mat, imgproc, contours, etc.


// --- Boost.Geometry (for union of cluster envelopes) ---
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace go2_control_cpp
{

// Short aliases for Boost.Geometry types
namespace bg = boost::geometry;
using BGPoint = bg::model::d2::point_xy<double>;
using BGPoly  = bg::model::polygon<BGPoint, false, true>;   // CCW, closed
using BGMPoly = bg::model::multi_polygon<BGPoly>;

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseStamped;


// ------------------------------ Math helpers -------------------------------
struct Vec2
{
  double x{0}, y{0};
  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}
  inline Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  inline Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  inline Vec2 operator*(double s)     const { return {x * s, y * s}; }
};

inline double dot(const Vec2& a, const Vec2& b){ return a.x*b.x + a.y*b.y; }
inline double norm(const Vec2& a){ return std::sqrt(dot(a,a)); }
inline Vec2   normalize(const Vec2& a){ double n = norm(a); return (n>1e-9)? Vec2{a.x/n, a.y/n} : Vec2{0,0}; }
inline double cross2(const Vec2& a, const Vec2& b){ return a.x*b.y - a.y*b.x; }

// ---- Axis-centering hit types (file scope) ----
enum class HitType { EXT, ENVELOPE };

struct Hit {
  Vec2   p;
  HitType type;
};


// ----------------------------- Debug structs -------------------------------
struct RectangleCorridor
{
  Vec2   center;       // world coords (m)
  Vec2   major_dir;    // unit vector (long axis)
  double half_len;     // meters
  double half_width;   // meters
};

struct DebugRect {
  Vec2   center;  // world coords
  double yaw;     // radians, world frame
  double sx;      // size along long axis (m)
  double sy;      // size along short axis (m)
};

// Red extension rectangle (projected row segment)
struct ExtRect {
  Vec2   center;   // world coords
  double length;   // along chosen major
  double width;    // across chosen minor
  Vec2   u_major{1,0}; // NEW: unit vector for this rect's long axis
};

// Bounding envelope for one cluster (in world frame)
struct Envelope {
  Vec2   center;
  double halfL; // along cluster major
  double halfW; // along cluster minor
};

// (Kept for compatibility if you ever need a single oriented box fast test)
struct EnvelopeProj {
  double ox{0}, oy{0}, res{1}; // map origin & resolution
  double cx{0}, cy{0};         // envelope center
  double ca{1}, sa{0};         // cos(yaw), sin(yaw) of envelope major axis
  double hL{0}, hW{0};         // half-length/width
  bool   valid{false};
};

// ----------------------------- Clustering (multi) ---------------------------
struct ClusterEx {
  // Members (red rectangles)
  std::vector<ExtRect> members;

  // Frozen projection frame captured at birth of the cluster
  Vec2 n_major{1,0};   // unit long-axis
  Vec2 n_minor{0,1};   // unit short-axis
  Vec2 ref0{0,0};      // reference point for (a,b) projections

  // Stats (meters) in the frozen frame; bounds expand by rect half-sizes
  double a_min{0}, a_max{0}, a_center{0};
  double b_min{0}, b_max{0}, b_center{0};

  // Bookkeeping
  bool         initialized{false};
  size_t       id{0};
  rclcpp::Time last_seen;
};

// --- Cluster helpers (declared here, defined in the .cpp) ---

// Project a world point into a cluster's (a,b) frame.
std::pair<double,double> projectAB(const ClusterEx& C, const Vec2& p);

// Recompute (a_min/max/center, b_min/max/center) from members.
void recomputeStats(ClusterEx& C);

// Initialize a new cluster from a seed rectangle and a frozen frame.
void initFromSeed(ClusterEx& C,
                  const ExtRect& seed,
                  const Vec2& n_major,
                  const Vec2& n_minor,
                  const Vec2& ref0,
                  size_t id,
                  const rclcpp::Time& now);

// If any member center is within 'dedupe_thresh_m' of 'rect.center', replace it.
bool tryDedupeReplace(ClusterEx& C, const ExtRect& rect, double dedupe_thresh_m);

// Membership test: minor-axis alignment + within an expanded major band.
bool fitsCluster(const ClusterEx& C,
                 const ExtRect& rect,
                 double major_tol_m,
                 double minor_tol_m);

// Assign a batch of rectangles to existing clusters (dedupe/fit) or create new ones.
void assignExtsToClusters(std::vector<ClusterEx>& clusters,
                          const std::vector<ExtRect>& exts,
                          const Vec2& n_major_new,
                          const Vec2& n_minor_new,
                          size_t& next_cluster_id,
                          double major_tol_m,
                          double minor_tol_m,
                          double dedupe_thresh_m,
                          const rclcpp::Time& now);

// Choose an "active" cluster for planning (e.g., the one that contains the robot).
std::optional<size_t> pickActiveCluster(const std::vector<ClusterEx>& clusters,
                                        const Vec2& robot_xy);

// Build a world-space envelope from a cluster's members and padding.
std::optional<Envelope> makeEnvelopeForCluster(const ClusterEx& C, double band_m);

// Draw a shallow cube/outline for a cluster envelope.
void addClusterPerimeterMarker(const ClusterEx& C,
                               const std::string& frame,
                               const rclcpp::Time& stamp,
                               int id,
                               visualization_msgs::msg::MarkerArray& out,
                               float r=1.0f, float g=0.85f, float b=0.0f, float a=0.35f);

// Re-estimate cluster frame (n_major/n_minor/ref0) from members' u_major & centers.
// ang_thresh_deg: only rotate if change exceeds this; alpha_* are [0..1] smoothing.
void retuneClusterFrameFromMembers(ClusterEx& C,
                                   double ang_thresh_deg = 1.0,
                                   double alpha_dir = 1.0,
                                   double alpha_pos = 1.0);

// ------------------------- Union-of-envelopes (polygon) --------------------
struct EnvelopeUnion {
  BGMPoly mp;                                   // union (possibly multiple polygons)
  std::vector<std::pair<Vec2,Vec2>> edges;      // outer-ring edges for fast ray hits
  bool valid{false};
};

// Build union of all cluster envelopes (with band_m padding per cluster)
EnvelopeUnion buildGlobalEnvelopeUnion(const std::vector<ClusterEx>& clusters,
                                       double band_m);

// Point-in-union (boundary counts as inside)
bool insideUnion(const EnvelopeUnion& EU, double wx, double wy);

// Distance to first exit along ray (if starting inside). Returns nullopt if none.
std::optional<double> distanceToExitAlongRay(const EnvelopeUnion& EU,
                                             const Vec2& p0,
                                             const Vec2& u,
                                             double maxd);

// RViz helper: draw union as LINE_LIST
void addEnvelopeUnionMarker(const EnvelopeUnion& EU,
                            const std::string& frame,
                            const rclcpp::Time& stamp,
                            int id,
                            visualization_msgs::msg::MarkerArray& out,
                            float r=0.0f, float g=1.0f, float b=0.0f, float a=0.9f);

// ------------------------------ BT Action Node ------------------------------
class FrontierExplore : public BT::SyncActionNode
{
public:
  FrontierExplore(const std::string& name, const BT::NodeConfiguration& config);
  ~FrontierExplore() override;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  template<typename T>
  T getOrDefault(const std::string& key, const T& def)
  {
    auto v = this->getInput<T>(key);
    return v ? v.value() : def;
  }

  bool getRobotXY(const std::string& global_frame,
                  const std::string& base_frame,
                  Vec2& out);

  // Optional: if you still use OpenCV-based fitting elsewhere.
  bool fitRectangle(
    const std::function<Vec2(int,int)>& mapToWorld,
    const std::vector<cv::Point>& comp_pts, // cell-space points (x=col, y=row)
    double res,
    double target_aspect,
    double aspect_tol,   // unused, kept for signature stability
    RectangleCorridor& out);

  // --- Members ---
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridors_pub_;
  std::thread         spin_thread_;
  std::atomic<bool>   spinning_{false};

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr      goal_pub_;

  std::mutex                   map_mtx_;
  std::optional<OccupancyGrid> latest_map_;

  std::vector<RectangleCorridor> corridors_;
  std::vector<DebugRect>         debug_rects_;

  // Multi-cluster state
  std::vector<ClusterEx> clusters_;
  size_t                 next_cluster_id_{1};

  bool corridor_mode_active_;
};

} // namespace go2_control_cpp