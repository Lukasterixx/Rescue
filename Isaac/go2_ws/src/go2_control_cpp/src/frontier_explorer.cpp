#include "go2_control_cpp/frontier_explorer.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
// File-local storage for the global (inflated) costmap
std::mutex g_costmap_mtx;
std::optional<nav_msgs::msg::OccupancyGrid> g_latest_costmap;
rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr g_costmap_sub;

// Helper: Squared norm for distance checks
inline double normSq(const go2_control_cpp::Vec2& v) {
  return v.x * v.x + v.y * v.y;
}

// Helper: Separating Axis Theorem (SAT) to check overlap between two rotated rectangles
bool checkRectOverlap(const go2_control_cpp::ExtRect& A, const go2_control_cpp::ExtRect& B) {
  const go2_control_cpp::Vec2 cA = A.center;
  const go2_control_cpp::Vec2 uA = go2_control_cpp::normalize(A.u_major);
  const go2_control_cpp::Vec2 vA = {-uA.y, uA.x};
  const double eA_long = A.length * 0.5;
  const double eA_wide = A.width * 0.5;

  const go2_control_cpp::Vec2 cB = B.center;
  const go2_control_cpp::Vec2 uB = go2_control_cpp::normalize(B.u_major);
  const go2_control_cpp::Vec2 vB = {-uB.y, uB.x};
  const double eB_long = B.length * 0.5;
  const double eB_wide = B.width * 0.5;

  const go2_control_cpp::Vec2 T = cB - cA;
  const std::array<go2_control_cpp::Vec2, 4> axes = {uA, vA, uB, vB};

  for (const auto& L : axes) {
    double dist = std::abs(go2_control_cpp::dot(T, L));
    double rA = std::abs(go2_control_cpp::dot(uA, L)) * eA_long +
                std::abs(go2_control_cpp::dot(vA, L)) * eA_wide;
    double rB = std::abs(go2_control_cpp::dot(uB, L)) * eB_long +
                std::abs(go2_control_cpp::dot(vB, L)) * eB_wide;

    if (dist > (rA + rB)) {
      return false;
    }
  }
  return true;
}
} // namespace

namespace go2_control_cpp {

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

FrontierExplore::FrontierExplore(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS qos(1);
  qos.transient_local();
  qos.reliable();
  map_sub_ = node_->create_subscription<OccupancyGrid>(
      "/map2d", qos,
      [this](const OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mtx_);
        latest_map_.reset();
        latest_map_.emplace(*msg);
      });

  if (!g_costmap_sub) {
    rclcpp::QoS gc_qos(1);
    gc_qos.reliable();
    g_costmap_sub = node_->create_subscription<OccupancyGrid>(
        "/global_costmap/costmap", gc_qos,
        [](const OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(g_costmap_mtx);
          g_latest_costmap = *msg;
        });
  }

  goal_pub_ = node_->create_publisher<PoseStamped>("/goal_pose", rclcpp::QoS(10));
  corridors_pub_ = node_->create_publisher<MarkerArray>("/corridors", rclcpp::QoS(10));

  spinning_.store(false);
  corridor_mode_active_ = false;
}

FrontierExplore::~FrontierExplore() {
  spinning_.store(false);
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

BT::PortsList FrontierExplore::providedPorts() {
  return {
      // Keep only TF frames in the BT ports so the tree can control coordinate systems
      BT::InputPort<std::string>("global_frame", "map2d", "Fixed frame"),
      BT::InputPort<std::string>("base_frame", "livox_frame", "Robot frame"),
  };
}

std::pair<double, double> projectAB(const ClusterEx& C, const Vec2& p) {
  const Vec2 v = p - C.ref0;
  return {dot(v, C.n_major), dot(v, C.n_minor)};
}

void recomputeStats(ClusterEx& C) {
  if (C.members.empty()) {
    C.a_min = C.a_max = C.a_center = 0;
    C.b_min = C.b_max = C.b_center = 0;
    C.initialized = false;
    return;
  }
  double amin = 1e9, amax = -1e9, bmin = 1e9, bmax = -1e9;
  for (const auto& e : C.members) {
    auto [a, b] = projectAB(C, e.center);
    amin = std::min(amin, a - 0.5 * e.length);
    amax = std::max(amax, a + 0.5 * e.length);
    bmin = std::min(bmin, b - 0.5 * e.width);
    bmax = std::max(bmax, b + 0.5 * e.width);
  }
  C.a_min = amin; C.a_max = amax; C.a_center = 0.5 * (amin + amax);
  C.b_min = bmin; C.b_max = bmax; C.b_center = 0.5 * (bmin + bmax);
}

void assignExtsToClusters(std::vector<ClusterEx>& clusters,
                          const std::vector<ExtRect>& exts,
                          const Vec2& n_major_new,
                          const Vec2& n_minor_new,
                          size_t& next_cluster_id,
                          double major_tol_m,
                          double minor_tol_m,
                          double dedupe_thresh_m,
                          const rclcpp::Time& now) {
  for (const auto& e : exts) {
    bool placed = false;
    for (auto& C : clusters) {
      if (!C.initialized) continue;
      for (auto& m : C.members) {
        if (normSq(m.center - e.center) < dedupe_thresh_m * dedupe_thresh_m) {
          m = e;
          C.last_seen = now;
          recomputeStats(C);
          placed = true;
          break;
        }
      }
      if (placed) break;
    }
    if (placed) continue;

    for (auto& C : clusters) {
      if (!C.initialized) continue;
      if (fitsCluster(C, e, major_tol_m, minor_tol_m)) {
        C.members.push_back(e);
        C.last_seen = now;
        recomputeStats(C);
        placed = true;
        break;
      }
    }
    if (placed) continue;

    ClusterEx fresh;
    fresh.n_major = normalize(n_major_new);
    fresh.n_minor = normalize(n_minor_new);
    fresh.ref0 = e.center;
    fresh.id = next_cluster_id++;
    fresh.initialized = true;
    fresh.last_seen = now;
    fresh.members.push_back(e);
    recomputeStats(fresh);
    clusters.push_back(std::move(fresh));
  }
}

bool fitsCluster(const ClusterEx& C, const ExtRect& rect, double major_tol_m, double minor_tol_m) {
  auto [a, b] = projectAB(C, rect.center);
  const bool minor_ok = std::abs(b - C.b_center) <= minor_tol_m;
  const double hL = 0.5 * rect.length;
  const double r_lo = a - hL / 2.0;
  const double r_hi = a + hL / 2.0;
  const double c_lo = C.a_min - major_tol_m;
  const double c_hi = C.a_max + major_tol_m;
  const bool major_overlap = (std::max(r_lo, c_lo) <= std::min(r_hi, c_hi));
  return minor_ok && major_overlap;
}

std::optional<Envelope> makeEnvelopeForCluster(const ClusterEx& C, double band_m) {
  if (!C.initialized || C.members.empty()) return std::nullopt;
  const double midA = 0.5 * (C.a_min + C.a_max);
  const double midB = 0.5 * (C.b_min + C.b_max);

  Envelope e;
  e.center = C.ref0 + C.n_major * midA + C.n_minor * midB;
  // Use 2.0 * band_m for padding along major axis ends
  e.halfL = std::max(1e-3, 0.5 * (C.a_max - C.a_min) + 2.0 * band_m);
  e.halfW = std::max(1e-3, 0.5 * (C.b_max - C.b_min) + band_m);
  return e;
}

EnvelopeUnion buildGlobalEnvelopeUnion(const std::vector<ClusterEx>& clusters, double band_m) {
  EnvelopeUnion EU;
  bool seeded = false;
  for (const auto& C : clusters) {
    if (!C.initialized || C.members.empty()) continue;
    auto env = makeEnvelopeForCluster(C, band_m);
    if (!env) continue;

    const Vec2 u = C.n_major, v = C.n_minor, cx = env->center;
    const double hL = env->halfL, hW = env->halfW;

    std::array<Vec2, 5> pts = {
        cx + u * hL + v * hW, cx - u * hL + v * hW,
        cx - u * hL - v * hW, cx + u * hL - v * hW,
        cx + u * hL + v * hW};

    BGPoly p;
    for (auto& q : pts) bg::append(p.outer(), BGPoint(q.x, q.y));
    bg::correct(p);

    if (!seeded) {
      EU.mp.push_back(p);
      seeded = true;
    } else {
      BGMPoly tmp;
      BGMPoly one;
      one.push_back(p);
      bg::union_(EU.mp, one, tmp);
      EU.mp = std::move(tmp);
    }
  }

  if (seeded) {
    for (const auto& poly : EU.mp) {
      const auto& ring = poly.outer();
      for (size_t i = 1; i < ring.size(); ++i) {
        EU.edges.emplace_back(Vec2{ring[i - 1].x(), ring[i - 1].y()},
                              Vec2{ring[i].x(), ring[i].y()});
      }
    }
    EU.valid = !EU.edges.empty();
  }
  return EU;
}

bool insideUnion(const EnvelopeUnion& EU, double wx, double wy) {
  if (!EU.valid) return true;
  BGPoint p(wx, wy);
  for (const auto& poly : EU.mp) {
    if (bg::covered_by(p, poly)) return true;
  }
  return false;
}

std::optional<double> distanceToExitAlongRay(const EnvelopeUnion& EU, const Vec2& p0, const Vec2& u, double maxd) {
  if (!EU.valid || !insideUnion(EU, p0.x, p0.y)) return std::nullopt;
  double best = 1e9;
  for (const auto& e : EU.edges) {
    const Vec2 a = e.first, b = e.second;
    const Vec2 r = u, s{b.x - a.x, b.y - a.y};
    const double rxs = cross2(r, s);
    if (std::abs(rxs) < 1e-12) continue;
    const Vec2 qmp{a.x - p0.x, a.y - p0.y};
    const double t = cross2(qmp, s) / rxs;
    const double useg = cross2(qmp, r) / rxs;
    if (t >= 0.0 && t <= maxd && useg >= 0.0 && useg <= 1.0 && t < best) {
      best = t;
    }
  }
  return std::isfinite(best) ? std::optional<double>(best) : std::nullopt;
}

static inline Vec2 perp(const Vec2& a) { return Vec2{-a.y, a.x}; }

void retuneClusterFrameFromMembers(ClusterEx& C, double ang_thresh_deg, double alpha_dir, double alpha_pos) {
  if (!C.initialized || C.members.empty()) return;
  Vec2 sum{0, 0};
  for (const auto& e : C.members) {
    Vec2 u = normalize(e.u_major);
    if (dot(u, C.n_major) < 0.0) u = u * (-1.0);
    sum = sum + u;
  }
  Vec2 avg_u = normalize(sum);
  if (norm(avg_u) < 1e-9) avg_u = C.n_major;

  double cosang = std::clamp(dot(C.n_major, avg_u), -1.0, 1.0);
  double dang = std::acos(cosang);
  if (dang > (ang_thresh_deg * M_PI / 180.0)) {
    C.n_major = normalize(C.n_major * (1.0 - alpha_dir) + avg_u * alpha_dir);
    C.n_minor = perp(C.n_major);
  }
  Vec2 ctr{0, 0};
  for (const auto& e : C.members) ctr = ctr + e.center;
  C.ref0 = C.ref0 * (1.0 - alpha_pos) + (ctr * (1.0 / C.members.size())) * alpha_pos;
  recomputeStats(C);
}

void addClusterPerimeterMarker(const ClusterEx& C, const std::string& frame, const rclcpp::Time& stamp, int id, visualization_msgs::msg::MarkerArray& out, float r, float g, float b, float a) {
  auto env = makeEnvelopeForCluster(C, 0.0);
  if (!env) return;
  const Vec2 u = C.n_major, v = C.n_minor, cx = env->center;
  const double hL = env->halfL, hW = env->halfW;
  std::array<Vec2, 5> pts = { cx + u*hL + v*hW, cx - u*hL + v*hW, cx - u*hL - v*hW, cx + u*hL - v*hW, cx + u*hL + v*hW };
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame; m.header.stamp = stamp; m.ns = "cluster_perimeter"; m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP; m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.06; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
  for (const auto& p : pts) { geometry_msgs::msg::Point q; q.x=p.x; q.y=p.y; q.z=0.07; m.points.push_back(q); }
  out.markers.push_back(std::move(m));
}

void addEnvelopeUnionMarker(const EnvelopeUnion& EU, const std::string& frame, const rclcpp::Time& stamp, int id, visualization_msgs::msg::MarkerArray& out, float r, float g, float b, float a) {
  if (!EU.valid) return;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame; m.header.stamp = stamp; m.ns = "envelope_union"; m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_LIST; m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.06; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
  auto toPt = [](const Vec2& p){ geometry_msgs::msg::Point q; q.x=p.x; q.y=p.y; q.z=0.07; return q; };
  for (const auto& e : EU.edges) { m.points.push_back(toPt(e.first)); m.points.push_back(toPt(e.second)); }
  out.markers.push_back(std::move(m));
}

// --------------------------------- Tick -----------------------------------
// --------------------------------- Tick -----------------------------------
BT::NodeStatus FrontierExplore::tick() {
  // 1. Get Frames from BT Ports
  const std::string global_frame = getOrDefault<std::string>("global_frame", "map2d");
  const std::string base_frame   = getOrDefault<std::string>("base_frame", "livox_frame");

  // 2. Get Algorithm Params
  auto getParam = [&](const std::string& key, auto default_val) {
    if (!node_->has_parameter(key)) {
      node_->declare_parameter(key, default_val);
    }
    auto val = default_val; 
    node_->get_parameter(key, val);
    return val;
  };

  const double radius_m         = getParam("radius_m", 30.0);
  const int min_rect_area       = getParam("min_rect_area", 60);
  const double aspect_min       = getParam("aspect_ratio", 1.5);
  const int occ_thresh          = getParam("occ_thresh", 65);
  const double backoff_m        = getParam("backoff_from_frontier_m", 0.6);
  const double corridor_margin  = getParam("corridor_margin_m", 0.5);
  const double ray_step_deg     = std::max(0.5, getParam("ray_step_deg", 3.0));
  const double panel_length_m   = std::max(0.1, getParam("panel_length_m", 70.0));
  const int infl_block_cost     = getParam("inflation_block_at_cost", 1);
  const double los_thickness_m  = std::max(0.0, getParam("los_thickness_m", 0.0));
  const double band_m           = std::max(0.0, getParam("band_m", 7.0));
  const double fov_half_deg     = std::clamp(getParam("fov_half_deg", 120.0), 0.0, 180.0);
  const double min_env_frontier_m = std::max(0.0, getParam("min_envelope_frontier_dist_m", 7.5));
  const bool center_use_quadrature = getParam("center_use_quadrature", true);
  const double los_min_frontier_dist_m = std::max(0.0, getParam("los_min_frontier_dist_m", 1.5));
  const double los_escape_inflation_m = std::max(0.0, getParam("los_escape_inflation_m", 1.5));
  bool corridor_mode = corridor_mode_active_;

  Vec2 robot_xy;
  if (!getRobotXY(global_frame, base_frame, robot_xy)) return BT::NodeStatus::FAILURE;
  double robot_yaw = 0.0;
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame, base_frame, tf2::TimePointZero);
    robot_yaw = tf2::getYaw(tf.transform.rotation);
  } catch (...) {}

  OccupancyGrid map;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    if (!latest_map_) return BT::NodeStatus::FAILURE;
    map = *latest_map_;
  }
  std::optional<OccupancyGrid> infl;
  {
    std::lock_guard<std::mutex> lk(g_costmap_mtx);
    if (g_latest_costmap) infl = *g_latest_costmap;
  }

  const double res = map.info.resolution, ox = map.info.origin.position.x, oy = map.info.origin.position.y;
  const int width = static_cast<int>(map.info.width), height = static_cast<int>(map.info.height);
  const double brush_r = los_thickness_m / res;
  const int brush_maxr = static_cast<int>(std::ceil(brush_r));

  auto worldToMap = [&](double wx, double wy, int& mx, int& my) {
    mx = static_cast<int>(std::floor((wx - ox) / res));
    my = static_cast<int>(std::floor((wy - oy) / res));
    return (mx >= 0 && mx < width && my >= 0 && my < height);
  };
  auto mapToWorld = [&](int mx, int my) {
    return Vec2{ox + (mx + 0.5) * res, oy + (my + 0.5) * res};
  };
  auto idx = [&](int x, int y) { return y * width + x; };

  auto isInflatedWorld = [&](double wx, double wy) {
    if (!infl) return false;
    int mx = static_cast<int>(std::floor((wx - infl->info.origin.position.x) / infl->info.resolution));
    int my = static_cast<int>(std::floor((wy - infl->info.origin.position.y) / infl->info.resolution));
    if (mx < 0 || my < 0 || mx >= (int)infl->info.width || my >= (int)infl->info.height) return false;
    return infl->data[my * (int)infl->info.width + mx] >= infl_block_cost;
  };

  int rx, ry; worldToMap(robot_xy.x, robot_xy.y, rx, ry);
  const int rad_cells = static_cast<int>(radius_m / res);
  const int xmin = std::max(0, rx - rad_cells), xmax = std::min(width - 1, rx + rad_cells);
  const int ymin = std::max(0, ry - rad_cells), ymax = std::min(height - 1, ry + rad_cells);

  // --- CV OPTIMIZATION START ---
  cv::Mat occ_full(ymax - ymin + 1, xmax - xmin + 1, CV_8UC1, cv::Scalar(0));
  for (int y = ymin; y <= ymax; ++y) {
    uint8_t* row_ptr = occ_full.ptr<uint8_t>(y - ymin);
    for (int x = xmin; x <= xmax; ++x) {
      if ((x - rx) * (x - rx) + (y - ry) * (y - ry) > rad_cells * rad_cells) continue;
      if (map.data[idx(x, y)] >= occ_thresh) row_ptr[x - xmin] = 255;
    }
  }

  double cv_scale = 0.25; 
  cv::Mat occ;
  if (occ_full.rows > 10 && occ_full.cols > 10) {
      cv::resize(occ_full, occ, cv::Size(), cv_scale, cv_scale, cv::INTER_NEAREST);
  } else {
      cv_scale = 1.0;
      occ = occ_full;
  }
  double effective_res = res / cv_scale;
  int k_size = static_cast<int>(1.0 / effective_res);
  if (k_size % 2 == 0) k_size++; 
  if (k_size < 3) k_size = 3;    

  cv::morphologyEx(occ, occ, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {k_size, k_size}));
  cv::morphologyEx(occ, occ, cv::MORPH_DILATE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(occ, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  corridors_.clear();
  debug_rects_.clear();

  double physical_area_thresh_m2 = (double)min_rect_area * (0.5 * 0.5); 
  if (physical_area_thresh_m2 < 2.0) physical_area_thresh_m2 = 2.0; 

  for (const auto& c : contours) {
    double area_m2 = cv::contourArea(c) * (effective_res * effective_res);
    if (area_m2 < physical_area_thresh_m2) continue;
    cv::RotatedRect r = cv::minAreaRect(c);
    double center_x_roi = r.center.x / cv_scale;
    double center_y_roi = r.center.y / cv_scale;
    double width_roi    = r.size.width / cv_scale;
    double height_roi   = r.size.height / cv_scale;
    double ls = std::max(width_roi, height_roi), ss = std::min(width_roi, height_roi);
    if (ls / ss < aspect_min) continue;
    const double yaw_w = (width_roi < height_roi) ? (r.angle + 90.0) * M_PI / 180.0 : r.angle * M_PI / 180.0;
    Vec2 ctr_w = mapToWorld(xmin + (int)center_x_roi, ymin + (int)center_y_roi);
    corridors_.push_back({ctr_w, {std::cos(yaw_w), -std::sin(yaw_w)}, 0.5 * ls * res, 0.5 * ss * res});
    debug_rects_.push_back({ctr_w, -yaw_w, ls * res, ss * res});
  }
  // --- CV OPTIMIZATION END ---

  double avg_yaw = 0.0;
  if (!corridors_.empty()) {
    Vec2 sum{0, 0}, ref = corridors_[0].major_dir;
    for (auto& cor : corridors_) {
      if (dot(cor.major_dir, ref) < 0) cor.major_dir = cor.major_dir * (-1);
      sum = sum + cor.major_dir;
    }
    avg_yaw = std::atan2(sum.y, sum.x);
  }
  Vec2 draw_major{std::cos(-avg_yaw), std::sin(-avg_yaw)}, draw_minor{-draw_major.y, draw_major.x};

  const double hard_half_cap = std::max(panel_length_m, radius_m * 2.0);
  auto cast_len = [&](const Vec2& origin, const Vec2& dir, double halfW) {
    Vec2 u = normalize(dir), n{-u.y, u.x};
    for (double s = 0.0; s <= hard_half_cap; s += res) {
      bool clear = true;
      for (double t = -halfW; t <= halfW; t += res) {
        Vec2 w = origin + u * s + n * t; int mx, my;
        if (worldToMap(w.x, w.y, mx, my) && map.data[idx(mx, my)] >= occ_thresh) {
          clear = false; break;
        }
      }
      if (clear) return s;
    }
    return hard_half_cap;
  };

  std::vector<ExtRect> exts;
  if (!corridors_.empty()) {
    Vec2 head{std::cos(robot_yaw), std::sin(robot_yaw)};
    if (std::abs(dot(head, draw_major)) >= std::sin(35.0 * M_PI / 180.0)) {
      for (const auto& cor : corridors_) {
        const double hW = cor.half_width + corridor_margin;
        double s_p = cast_len(cor.center, draw_major, hW), s_m = cast_len(cor.center, draw_major * (-1), hW);
        double l = s_m, r = s_p;
        if (l + r > panel_length_m) {
          double ex = (l + r) - panel_length_m;
          if (dot(head, draw_major) >= 0) l = std::max(0.0, l - ex); else r = std::max(0.0, r - ex);
        } else {
          double rem = panel_length_m - (l + r);
          if (dot(head, draw_major) >= 0) r += rem; else l += rem;
        }
        ExtRect cand{cor.center + draw_major * (0.5 * (r - l)), r + l, 2.0 * hW, draw_major};
        bool coll = false;
        for (const auto& C : clusters_) {
          for (const auto& m : C.members) {
            if (checkRectOverlap(cand, m) && normSq(cand.center - m.center) > 7.0 * 7.0) coll = true;
          }
        }
        if (!coll) exts.push_back(cand);
      }
    }
  }

  if (!exts.empty()) {
    assignExtsToClusters(clusters_, exts, draw_major, draw_minor, next_cluster_id_, 2.0, 200.0, 7.0, node_->now());
    for (auto& C : clusters_) {
      if (C.initialized && (node_->now() - C.last_seen).seconds() < 0.25) retuneClusterFrameFromMembers(C, 0.25, 1.0, 1.0);
    }
  }

  EnvelopeUnion envU = buildGlobalEnvelopeUnion(clusters_, band_m);
  std::optional<Envelope> env;
  if (!exts.empty()) {
    const Vec2 ref0 = exts.front().center;
    double minA = 1e9, maxA = -1e9, minB = 1e9, maxB = -1e9;
    for (const auto& E : exts) {
      Vec2 v = E.center - ref0; double a = dot(v, draw_major), b = dot(v, draw_minor);
      minA = std::min(minA, a - 0.5 * E.length); maxA = std::max(maxA, a + 0.5 * E.length);
      minB = std::min(minB, b - 0.5 * E.width); maxB = std::max(maxB, b + 0.5 * E.width);
    }
    env = {ref0 + draw_major * (0.5 * (minA + maxA)) + draw_minor * (0.5 * (minB + maxB)), 0.5 * (maxA - minA) + 2.0 * band_m, 0.5 * (maxB - minB) + band_m};
  }

  {
    MarkerArray arr;
    const rclcpp::Time now = node_->now();
    Marker clear; clear.header.frame_id = global_frame; clear.header.stamp = now; clear.action = Marker::DELETEALL; arr.markers.push_back(clear);
    for (size_t i = 0; i < debug_rects_.size(); ++i) {
      const auto& R = debug_rects_[i];
      Marker m; m.header.frame_id = global_frame; m.header.stamp = now; m.ns = "detected_rects"; m.id = static_cast<int>(i);
      m.type = Marker::CUBE; m.pose.position.x = R.center.x; m.pose.position.y = R.center.y; m.pose.position.z = 4.0;
      tf2::Quaternion q; q.setRPY(0, 0, -R.yaw); m.pose.orientation = tf2::toMsg(q);
      m.scale.x = R.sx; m.scale.y = R.sy; m.scale.z = 0.4; m.color.r = 0.0f; m.color.g = 0.8f; m.color.b = 1.0f; m.color.a = 0.35f;
      arr.markers.push_back(m);
    }
    for (size_t i = 0; i < exts.size(); ++i) {
      const auto& E = exts[i];
      Marker m; m.header.frame_id = global_frame; m.header.stamp = now; m.ns = "extended_rects"; m.id = static_cast<int>(i);
      m.type = Marker::CUBE; m.pose.position.x = E.center.x; m.pose.position.y = E.center.y; m.pose.position.z = 0.08;
      tf2::Quaternion q; q.setRPY(0, 0, -avg_yaw); m.pose.orientation = tf2::toMsg(q);
      m.scale.x = E.length; m.scale.y = E.width; m.scale.z = 0.4; m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.45f;
      arr.markers.push_back(m);
    }
    if (env) {
      Marker m; m.header.frame_id = global_frame; m.header.stamp = now; m.ns = "envelope_outline"; m.id = 0;
      m.type = Marker::LINE_STRIP; m.scale.x = 0.06; m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.85f;
      const Vec2 cx = env->center, u = draw_major, v = draw_minor; const double hL = env->halfL, hW = env->halfW;
      std::array<Vec2, 5> corners = { cx + u*hL + v*hW, cx - u*hL + v*hW, cx - u*hL - v*hW, cx + u*hL - v*hW, cx + u*hL + v*hW };
      for (const auto& p : corners) { geometry_msgs::msg::Point q; q.x=p.x; q.y=p.y; q.z=0.07; m.points.push_back(q); }
      arr.markers.push_back(m);
    }
    for (size_t i = 0; i < clusters_.size(); ++i) addClusterPerimeterMarker(clusters_[i], global_frame, now, static_cast<int>(i), arr);
    addEnvelopeUnionMarker(envU, global_frame, now, 0, arr, 0.0f, 1.0f, 0.0f, 0.9f);
    corridors_pub_->publish(arr);
  }

  std::vector<int8_t> work = map.data;
  if (!exts.empty()) {
    for (int y = ymin; y <= ymax; ++y) {
      for (int x = xmin; x <= xmax; ++x) {
        Vec2 wc = mapToWorld(x, y);
        for (const auto& E : exts) {
          Vec2 v = wc - E.center;
          if (std::abs(dot(v, E.u_major)) <= 0.5 * E.length && std::abs(dot(v, perp(E.u_major))) <= 0.5 * E.width) {
            work[idx(x, y)] = 100; break;
          }
        }
      }
    }
  }

  auto isUnknownOrig = [&](int x, int y) { return map.data[idx(x, y)] < 0; };
  auto isOccW_noEnv = [&](int x, int y) {
    if (work[idx(x, y)] >= occ_thresh) return true;
    Vec2 wc = mapToWorld(x, y); return isInflatedWorld(wc.x, wc.y);
  };
  auto isOccWThick_noEnv = [&](int cx, int cy) {
    if (brush_r <= 1e-9) return isOccW_noEnv(cx, cy);
    for (int dy = -brush_maxr; dy <= brush_maxr; ++dy) for (int dx = -brush_maxr; dx <= brush_maxr; ++dx) {
      if (dx * dx + dy * dy > brush_r * brush_r) continue;
      int xx = cx + dx, yy = cy + dy; if (xx < 0 || yy < 0 || xx >= width || yy >= height) continue;
      if (isOccW_noEnv(xx, yy)) return true;
      if (envU.valid && !insideUnion(envU, ox + (xx + 0.5) * res, oy + (yy + 0.5) * res)) return true;
    }
    return false;
  };

  auto pointInAnyExt = [&](const Vec2& p)->bool {
    for (const auto& E : exts) {
      Vec2 v = p - E.center; Vec2 n_mj = normalize(E.u_major), n_mn = perp(n_mj);
      if (std::abs(dot(v, n_mj)) <= 0.5 * E.length && std::abs(dot(v, n_mn)) <= 0.5 * E.width) return true;
    }
    return false;
  };

  // --- UPDATED RAYCASTER: Accepts "permissive" flag ---
  auto cast_one = [&](double theta, bool permissive) -> std::optional<Vec2> {
    Vec2 u{std::cos(theta), std::sin(theta)};
    int x0, y0, x1, y1; worldToMap(robot_xy.x, robot_xy.y, x0, y0);
    Vec2 p_end = robot_xy + u * radius_m; worldToMap(p_end.x, p_end.y, x1, y1);
    int x = x0, y = y0, dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1, dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1, err = dx + dy;
    const double skip = isInflatedWorld(robot_xy.x, robot_xy.y) ? los_escape_inflation_m : 0.0;
    
    while (true) {
      if (x == x1 && y == y1) break;
      int e2 = 2 * err, nx = x, ny = y;
      if (e2 >= dy) { err += dy; nx += sx; } if (e2 <= dx) { err += dx; ny += sy; }
      
      // -- Standard strict checks (skip if permissive) --
      if (!permissive) {
        if (!corridor_mode) {
            if (normSq(mapToWorld(x, y) - robot_xy) >= skip * skip && isOccWThick_noEnv(x, y)) return std::nullopt;
            if (normSq(mapToWorld(nx, ny) - robot_xy) >= skip * skip && isOccWThick_noEnv(nx, ny)) return std::nullopt;
            if (nx != x && ny != y && (isOccWThick_noEnv(nx, y) || isOccWThick_noEnv(x, ny))) return std::nullopt;
        }
      }

      Vec2 wnext = mapToWorld(nx, ny), wcur = mapToWorld(x, y);

      // -- Extension Rect check (skip if permissive) --
      if (!permissive) {
          if (pointInAnyExt(wnext)) {
            return std::nullopt;
          }
      }

      // -- Envelope checks (kept loosely, but primary permissive goal implies ignoring blocks)
      if (!permissive && envU.valid && insideUnion(envU, wcur.x, wcur.y) && !insideUnion(envU, wnext.x, wnext.y)) {
        if (!(nx >= 0 && nx < width && ny >= 0 && ny < height && isUnknownOrig(nx, ny))) {
          auto d_exit = distanceToExitAlongRay(envU, robot_xy, u, radius_m);
          double d = d_exit.value_or(norm(wcur - robot_xy));
          if (d >= min_env_frontier_m) return robot_xy + u * d;
          return std::nullopt;
        }
      }
      
      // -- Frontier Detection (ALWAYS ACTIVE) --
      if (nx >= 0 && nx < width && ny >= 0 && ny < height && isUnknownOrig(nx, ny)) {
        // If permissive, we ignore inflated world checks on the frontier point itself too
        bool point_valid = true;
        if (!permissive) {
            if (isInflatedWorld(wnext.x, wnext.y)) point_valid = false;
            if (envU.valid && !insideUnion(envU, wnext.x, wnext.y)) point_valid = false;
        }
        
        if (point_valid) {
          if (normSq(wnext - robot_xy) >= los_min_frontier_dist_m * los_min_frontier_dist_m) return wnext;
        }
      }
      x = nx; y = ny;
    }
    return std::nullopt;
  };

  // --- SEARCH HELPER ---
  auto run_search = [&](double fov_deg, bool permissive) -> std::optional<Vec2> {
      const double step = ray_step_deg * M_PI / 180.0;
      const double fov_limit = fov_deg * M_PI / 180.0;
      for (int k = 0; k <= static_cast<int>(fov_limit / step); ++k) {
        for (int sign : {1, -1}) {
          double th = robot_yaw + sign * static_cast<double>(k) * step;
          auto hit = cast_one(th, permissive); 
          if (hit) return hit; 
          if (k == 0) break; 
        }
      }
      return std::nullopt;
  };

  // 1. Primary Search (Standard FOV, Strict obstacles)
  std::optional<Vec2> best_f = run_search(fov_half_deg, false);

  // 2. Fallback Search (360 deg, Ignore obstacles/rects)
  if (!best_f) {
      best_f = run_search(180.0, true);
  }

  if (!best_f) return BT::NodeStatus::FAILURE;

  Vec2 frontier = *best_f, approach = frontier - normalize(frontier - robot_xy) * backoff_m;
  bool is_sh = false, next_tick_corr = false;

  if (!exts.empty() || envU.valid) {
    enum class HitType { EXT, ENVELOPE };
    struct CenteringHit { Vec2 p; HitType type; };
    auto f_hit = [&](const Vec2& p0, const Vec2& dir) -> std::optional<CenteringHit> {
      Vec2 u = normalize(dir);
      for (double s = 0; s <= 10.0; s += 0.02) {
        Vec2 p = p0 + u * s;
        for (const auto& E : exts) {
          Vec2 v = p - E.center;
          if (std::abs(dot(v, E.u_major)) <= 0.5 * E.length + 1e-3 && std::abs(dot(v, perp(E.u_major))) <= 0.5 * E.width + 1e-3) return CenteringHit{p, HitType::EXT};
        }
        if (envU.valid && !insideUnion(envU, p.x, p.y)) return CenteringHit{p, HitType::ENVELOPE};
      }
      return std::nullopt;
    };
    double c_y = avg_yaw; if (center_use_quadrature && std::abs(std::sin(robot_yaw - avg_yaw)) > std::abs(std::sin(robot_yaw - (avg_yaw + M_PI_2)))) c_y += M_PI_2;
    Vec2 a_mj{std::cos(-c_y), std::sin(-c_y)}, a_mn = perp(a_mj);
    auto h_n = f_hit(approach, a_mn * -1.0), h_p = f_hit(approach, a_mn);
    if (h_n && h_p) {
      Vec2 mid = (h_n->p + h_p->p) * 0.5; int mx, my;
      if (worldToMap(mid.x, mid.y, mx, my) && map.data[idx(mx, my)] < occ_thresh) {
        approach = mid; is_sh = true; if (h_n->type == HitType::EXT && h_p->type == HitType::EXT) next_tick_corr = true;
      }
    }
    auto p_n = f_hit(approach, a_mj * -1.0), p_p = f_hit(approach, a_mj);
    if (p_n && p_p) {
      Vec2 mid = (p_n->p + p_p->p) * 0.5; int mx, my;
      if (worldToMap(mid.x, mid.y, mx, my) && map.data[idx(mx, my)] < occ_thresh) { approach = mid; is_sh = true; }
    }
  }

  corridor_mode_active_ = next_tick_corr;
  PoseStamped goal; goal.header.stamp = node_->now(); goal.header.frame_id = global_frame;
  goal.pose.position.x = approach.x; goal.pose.position.y = approach.y;
  double t_y = is_sh ? (std::abs(std::remainder(robot_yaw - avg_yaw, 2 * M_PI)) < M_PI_2 ? avg_yaw : avg_yaw + M_PI) : std::atan2(frontier.y - approach.y, frontier.x - approach.x);
  tf2::Quaternion q; q.setRPY(0, 0, t_y); goal.pose.orientation = tf2::toMsg(q);
  goal_pub_->publish(goal);

  return BT::NodeStatus::SUCCESS;
}

bool FrontierExplore::getRobotXY(const std::string& global_frame, const std::string& base_frame, Vec2& out) {
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame, base_frame, tf2::TimePointZero);
    out.x = tf.transform.translation.x; out.y = tf.transform.translation.y;
    return true;
  } catch (...) { return false; }
}

} // namespace go2_control_cpp

BT_REGISTER_NODES(factory) {
  factory.registerBuilder<go2_control_cpp::FrontierExplore>(
      "FrontierExplore",
      [](auto& name, auto& config) {
        return std::make_unique<go2_control_cpp::FrontierExplore>(name, config);
      });
}