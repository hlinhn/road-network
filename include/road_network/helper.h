#ifndef ROAD_NETWORK_HELPER_H_
#define ROAD_NETWORK_HELPER_H_

#include <opencv2/core/types.hpp>
#include <optional>
#include <string>

struct RouteGraphConfig
{
  std::string image_path;
  std::string bag_path;
  std::string inner_image_path;
  std::string save_folder;
  double original_resolution;
  double resize_ratio;

  std::string topic_name;
  double skip_distance;
  double candidate_region_size;
  int octree_resolution;
  int edge_threshold;

  bool print_debug;
  bool save_inner_region;
  bool save_inner_region_original;

  int blur_kernel; // 3
  int laplacian_kernel; // 3
  int laplacian_scale; // 1
  int laplacian_delta; // 0

  double disc_min_radius; // 3.0
  double fudge_factor; // 0.5
  double size_fudge_factor; // 0.6
  double same_size_factor; // 0.1

  bool save_covered_image;

  int max_neighbors; // 10
  double fudge_intersect_factor; // 1.1
  double cosine_threshold; // 0.95

  bool save_proposed_edge_image;

  std::string save_graph_path;

  bool mark_inner_region;
  bool generate_graph;
};

struct Disc
{
  Disc(cv::Point2i c, double r)
    : center {c}
    , radius {r}
  {
  }
  cv::Point2i center;
  double radius;
};

struct Edge
{
  Edge(int i, int j)
    : from {i}
    , to {j}
  {
  }
  int from;
  int to;
};

double distance(cv::Point2i p, cv::Point2i q);
cv::Point2i convertToIndex(const double x, const double y, const double resolution, const cv::Size image_size);
RouteGraphConfig readConfig(std::string config_name);
std::vector<cv::Point2i> parseBagPath(std::string bag_path,
                                      cv::Size image_size,
                                      double resolution,
                                      double skip_distance,
                                      std::string topic);

double cosAngle(std::vector<Disc> discs, Edge e, Edge f);

void saveGraph(std::vector<Disc> discs, std::vector<Edge> edges, std::string path);

#endif
