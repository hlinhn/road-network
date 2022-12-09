#ifndef ROAD_NETWORK_DETECT_ROUTE_GRAPH_H_
#define ROAD_NETWORK_DETECT_ROUTE_GRAPH_H_

#include <opencv2/imgcodecs.hpp>
#include <road_network/detect_route_graph.h>
#include <road_network/helper.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointChecker
{
public:
  PointChecker(cv::Mat image, int threshold);
  std::optional<cv::Point2i> checkIntersection(cv::Point2i start, cv::Point2i end);

private:
  cv::Mat image_;
  int threshold_;

  bool outOfRange(const cv::Point2i index);
  std::optional<cv::Vec<uint16_t, 4>> queryPoint(const cv::Point2i index);
  bool checkPoint(const cv::Point2i index);
};

class ClassifyRegion
{
public:
  ClassifyRegion(RouteGraphConfig config);
  cv::Mat markInnerRegion(std::string image_path);
  std::vector<Edge> buildGraph(std::vector<Disc> discs, cv::Mat inner_region);
  std::vector<Disc> coverInnerRegion(cv::Mat inner_region);

private:
  RouteGraphConfig config_;
  cv::Mat classifyInnerRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr path_points,
                              cv::Mat candidates,
                              PointChecker point_checker);
};

#endif
