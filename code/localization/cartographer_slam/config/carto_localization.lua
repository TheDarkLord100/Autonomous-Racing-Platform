-- Pure localization against a frozen .pbstream (built with clean odometry),
-- tracking with noisy odometry.

include "carto_mapping.lua"

-- stock localization overrides (cartographer_ros backpack_2d_localization.lua)
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

-- noisy odometry: heading from the scans, not the odometry
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true   -- +-20 deg search around odometry
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.       -- stock 40; 1 snaps at hard corners
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.    -- stock 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.       -- stock 1e5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.   -- stock; mapping uses 2 m (clean odometry)

return options