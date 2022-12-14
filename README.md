# Generate road network

This package generates simple road network from edge image

What it does:
- classify inner and outer region of the road (useful for drivable map generation) (enabled with `mark_inner_region` in the config file)
- cover the inner region with discs and generate route graph (Reference: Reconstructing Road Network Graphs from both Aerial Lidar and Images) (enabled with `generate_graph` in the config file)

The route graph is still very simple (not directed, not free of cycles)

Explanation of some parameters:
- `bag_path`: this is the trajectory used to propose candidates for inner region image. Should contain `topic_name`, expected to be of type `Odometry`
- `inner_image_path`: this is the input to graph generation step, which could be the output of the first step or output of the first step with some modification (erosion, manual cleanup, etc)
