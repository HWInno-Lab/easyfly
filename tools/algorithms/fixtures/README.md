# Extension software fixtures

Excluded from default workspace discovery by `COLCON_IGNORE`. Build a named child package explicitly when reproducing the extension checks.

`easyfly_ext_validation_env` only exports the path of an explicitly prepared, wheel-hashed jsonschema dependency directory. It supplies no ROS nodes, algorithm implementations or synthetic package availability. It lets installed-prefix tests clear their environment and reload actual declared Python dependencies without modifying the user's system or user-site packages. Never install it into a production deployment prefix.
