-- Mapping with noisy odometry: same as carto_mapping.lua, but heading and position come
-- from the scans, not the odometry (the overrides carto_localization.lua already uses).

include "carto_mapping.lua"

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true   -- +-20 deg search around odometry
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.       -- stock 40
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.    -- stock 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.       -- stock 1e5

return options