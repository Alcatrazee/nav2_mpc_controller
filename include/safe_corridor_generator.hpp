#ifndef NAV2_MPC_CONTROLLER__SAFE_CORRIDOR_GENERATOR_HPP_
#define NAV2_MPC_CONTROLLER__SAFE_CORRIDOR_GENERATOR_HPP_

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include "trajectory_profiler.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_mpc_controller
{

/**
 * @brief Frenet and Cartesian corridor boundary definition at a specific path cross-section
 */
struct CorridorBound
{
  double s{0.0};           // Longitudinal arc length along reference path [m]
  double d_min{-0.8};      // Right lateral boundary in Frenet frame (<= 0) [m]
  double d_max{0.8};       // Left lateral boundary in Frenet frame (>= 0) [m]
  
  // Reference path point properties
  double x{0.0};           // Reference center point X [m]
  double y{0.0};           // Reference center point Y [m]
  double theta{0.0};       // Reference center point heading [rad]
  double kappa{0.0};       // Reference center point curvature [1/m]
  double v{0.0};           // Target velocity [m/s]
  double t{0.0};           // Relative time [s]

  // Corresponding Cartesian boundary coordinates
  double left_x{0.0};      // Cartesian Left boundary point X
  double left_y{0.0};      // Cartesian Left boundary point Y
  double right_x{0.0};     // Cartesian Right boundary point X
  double right_y{0.0};     // Cartesian Right boundary point Y
};

/**
 * @brief Complete safe corridor structure containing all cross-sections
 */
struct SafeCorridor
{
  std::vector<CorridorBound> bounds;
  std::string frame_id;
  rclcpp::Time stamp;

  bool empty() const { return bounds.empty(); }
  size_t size() const { return bounds.size(); }
  void clear() { bounds.clear(); }
};

/**
 * @brief Parameters configuration for safe corridor generation
 */
struct SafeCorridorConfig
{
  double default_left_width{0.8};        // Default maximum left corridor width [m]
  double default_right_width{0.8};       // Default maximum right corridor width [m]
  double min_corridor_width{0.15};       // Minimum allowable corridor width [m]
  double curvature_safety_factor{0.85};  // Safety factor alpha < 1.0 to avoid curvature center singularity
  double rib_safety_margin{0.05};        // Extra clearance margin when clipping cross-section intersections [m]
  double max_lateral_rate{0.4};          // Maximum lateral expansion/shrink rate tan(phi) along s
  double costmap_ray_step{0.02};         // Resolution for obstacle ray-casting in costmap [m] (2cm 高精采样)
  uint8_t costmap_cost_threshold{100};   // Costmap obstacle threshold (>= is treated as obstacle, default 100 includes inflation)
  bool check_costmap{true};              // Whether to query costmap for obstacle boundaries
  bool avoid_hairpin_self_intersection{true}; // Whether to detect and eliminate U-turn/hairpin corridor self-intersections
  int max_intersection_iters{3};         // Max iterations for non-adjacent self-intersection resolution
};

/**
 * @class SafeCorridorGenerator
 * @brief Generates collision-free, non-self-intersecting safe driving corridors along a reference trajectory.
 *        Specifically handles sharp curves and U-turn / hairpin bends to prevent geometric inversion and corridor overlap.
 */
class SafeCorridorGenerator
{
public:
  explicit SafeCorridorGenerator(const SafeCorridorConfig & config = SafeCorridorConfig());
  ~SafeCorridorGenerator() = default;

  /**
   * @brief Update generator configuration
   */
  void setConfig(const SafeCorridorConfig & config);

  /**
   * @brief Get current configuration
   */
  const SafeCorridorConfig & getConfig() const { return config_; }

  /**
   * @brief Build a safe corridor along given reference trajectory points
   * @param ref_points Parameterized reference trajectory points
   * @param costmap Optional pointer to Nav2 Costmap2D for obstacle detection
   * @param frame_id Coordinate frame ID
   * @param stamp Timestamp
   * @return SafeCorridor The constructed non-self-intersecting safe corridor
   */
  SafeCorridor generateCorridor(
    const std::vector<TrajectoryPoint> & ref_points,
    const nav2_costmap_2d::Costmap2D * costmap = nullptr,
    const std::string & frame_id = "map",
    const rclcpp::Time & stamp = rclcpp::Time());

  /**
   * @brief Create comprehensive RViz MarkerArray visualization for the safe corridor
   * @param corridor The safe corridor to visualize
   * @return visualization_msgs::msg::MarkerArray MarkerArray containing 3D mesh, boundary lines, and cross-section ribs
   */
  visualization_msgs::msg::MarkerArray createVisualizationMarkers(
    const SafeCorridor & corridor) const;

private:
  SafeCorridorConfig config_;

  // Step 1: Initialize bounds and perform 2D ray-casting on Costmap
  void initializeWithCostmap(
    SafeCorridor & corridor,
    const nav2_costmap_2d::Costmap2D * costmap) const;

  // Step 2: Apply curvature singularity limitation (1 - kappa * d > 0)
  void applyCurvatureLimit(SafeCorridor & corridor) const;

  // Step 3: Prevent adjacent cross-section rib intersection in tight turns
  void preventAdjacentRibIntersection(SafeCorridor & corridor) const;

  // Step 4: Detect and eliminate global / non-adjacent rib and boundary self-intersections (Hairpin turns)
  void preventHairpinSelfIntersection(SafeCorridor & corridor) const;

  // Step 5: Smooth corridor bounds using forward-backward lateral rate limiter
  void smoothCorridorBounds(SafeCorridor & corridor) const;

  // Step 6: Compute Cartesian coordinates for left and right boundary points
  void updateCartesianBoundaries(SafeCorridor & corridor) const;

  // Helper: 2D line segment intersection
  static bool computeLineSegmentIntersection(
    double p1_x, double p1_y, double p2_x, double p2_y,
    double p3_x, double p3_y, double p4_x, double p4_y,
    double & intersect_x, double & intersect_y,
    double & t1, double & t2);

  // Helper: 2D infinite line intersection
  static bool computeLineIntersection(
    double p1_x, double p1_y, double dir1_x, double dir1_y,
    double p2_x, double p2_y, double dir2_x, double dir2_y,
    double & lambda1, double & lambda2);
};

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__SAFE_CORRIDOR_GENERATOR_HPP_
