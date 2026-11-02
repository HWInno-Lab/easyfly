# Geometry remains useful when semantic_source is invalid; association IDs are aliases
# resolved ONLY inside semantic_source. Never treat geometry.target_id as a global key.
std_msgs/Header header
ClassifiedObstacleArray geometry
WorldTargetArrayV2 semantic_source
