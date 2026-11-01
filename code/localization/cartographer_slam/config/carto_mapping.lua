-- Stock Cartographer 2D. Only the options this car and simulator need are set;
-- everything else is the cartographer default (map_builder.lua,
-- trajectory_builder.lua, pose_graph.lua).

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- frames
  map_frame = "carto_map",                       -- own frame; the sim already owns "map"
  tracking_frame = "ego_racecar/base_link",      -- = child_frame_id of the odometry
  published_frame = "ego_racecar/base_link",
  odom_frame = "carto_odom",                     -- unused (provide_odom_frame = false)
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  -- outputs
  publish_to_tf = false,                         -- sim bridge publishes map -> base_link
  publish_tracked_pose = true,                   -- /tracked_pose, read by carto_node
  use_pose_extrapolator = false,                 -- tracked_pose = scan-matched pose at scan time

  -- sensors
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- timing / sampling (stock example values)
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false       -- no IMU
TRAJECTORY_BUILDER_2D.min_range = 0.1            -- m
TRAJECTORY_BUILDER_2D.max_range = 10.0           -- m, real RPLiDAR C1 is 12 m
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30   -- F: stock 90  
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.   -- stock 7 m reaches the parallel section ~7 m away
return options