# Generate road network

This package generates simple road network from edge image

What it does:
- classify inner and outer region of the road (useful for drivable map generation)
- cover the inner region with discs and generate route graph (Reference: Reconstructing Road Network Graphs from both Aerial Lidar and Images)

The route graph is still very simple (not directed, not free of cycles)
