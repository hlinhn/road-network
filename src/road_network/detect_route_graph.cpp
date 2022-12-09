#include <boost/filesystem.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/octree/octree_search.h>

#include <road_network/detect_route_graph.h>
#include <road_network/helper.h>

PointChecker::PointChecker(cv::Mat image, int threshold)
  : image_(image)
  , threshold_ {threshold}
{
}

bool
PointChecker::outOfRange(const cv::Point2i index)
{
  return (index.x >= image_.cols || index.x < 0 || index.y >= image_.rows || index.y < 0);
}

std::optional<cv::Vec<uint16_t, 4>>
PointChecker::queryPoint(const cv::Point2i index)
{
  if (outOfRange(index))
  {
    return std::nullopt;
  }
  return image_.at<cv::Vec<uint16_t, 4>>(index.y, index.x);
}

bool
PointChecker::checkPoint(const cv::Point2i index)
{
  for (int i = -1; i < 2; i++)
  {
    auto value_optional = queryPoint({index.x + i, index.y + i});
    if (!value_optional)
    {
      continue;
    }
    auto value = value_optional.value();
    if (value[0] > threshold_)
    {
      return true;
    }
  }
  for (int i = -1; i < 2; i++)
  {
    auto value_optional = queryPoint({index.x + i, index.y - i});
    if (!value_optional)
    {
      continue;
    }
    auto value = value_optional.value();
    if (value[0] > threshold_)
    {
      return true;
    }
  }
  return false;
}

std::optional<cv::Point2i>
PointChecker::checkIntersection(const cv::Point2i start, const cv::Point2i end)
{
  auto max_dx = end.x - start.x;
  auto max_dy = end.y - start.y;
  const auto sx = max_dx > 0 ? 1 : -1;
  const auto sy = max_dy > 0 ? 1 : -1;
  auto dx = std::abs(max_dx);
  auto dy = -std::abs(max_dy);
  cv::Point2i index = start;
  auto error = dx + dy;

  while (true)
  {
    if (outOfRange(index))
    {
      break;
    }
    if (checkPoint(index))
    {
      return index;
    }
    if (index.x == end.x && index.y == end.y)
    {
      break;
    }
    const auto error_2 = 2 * error;
    if (error_2 >= dy)
    {
      if (index.x == end.x)
      {
        break;
      }
      error += dy;
      index.x += sx;
    }
    if (error_2 <= dx)
    {
      if (index.y == end.y)
      {
        break;
      }
      error += dx;
      index.y += sy;
    }
  }
  return std::nullopt;
}

ClassifyRegion::ClassifyRegion(RouteGraphConfig config)
  : config_ {config}
{
}

cv::Mat
ClassifyRegion::classifyInnerRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr path_points,
                                    cv::Mat candidates,
                                    PointChecker point_checker)
{
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(config_.octree_resolution);
  octree.setInputCloud(path_points);
  octree.addPointsFromInputCloud();
  cv::Mat inner_region(candidates.size(), CV_8U, cv::Scalar(0));

  int count_processing_points = 0;
  int count_inner_candidates = 0;
  for (int i = 0; i < candidates.cols; i++)
  {
    for (int j = 0; j < candidates.rows; j++)
    {
      if (candidates.at<unsigned char>(j, i) > 0)
      {
        count_inner_candidates++;
        pcl::PointXYZ search_point(static_cast<float>(i), static_cast<float>(j), 0.0F);
        std::vector<int> point_indices;
        std::vector<float> point_distance;
        if (octree.nearestKSearch(search_point, 1, point_indices, point_distance) > 0)
        {
          cv::Point2i nearest;
          nearest.x = path_points->points[point_indices[0]].x;
          nearest.y = path_points->points[point_indices[0]].y;

          if (!(point_checker.checkIntersection(cv::Point2i {i, j}, nearest)))
          {
            inner_region.at<unsigned char>(j, i) = 255;
            count_processing_points++;
          }
        }
      }
    }
  }

  if (config_.print_debug)
  {
    std::cout << "Number of candidates: " << count_inner_candidates << std::endl;
    std::cout << "Number of actual points in inner region: " << count_processing_points << std::endl;
  }
  return inner_region;
}

std::vector<Disc>
ClassifyRegion::coverInnerRegion(cv::Mat inner_region)
{
  std::cout << "Find coverage" << std::endl;
  cv::Mat blurred;
  cv::GaussianBlur(inner_region, blurred, cv::Size(config_.blur_kernel, config_.blur_kernel), 0);
  cv::Mat edge;
  cv::Laplacian(blurred, edge, CV_8U, config_.laplacian_kernel, config_.laplacian_scale, config_.laplacian_delta);

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < edge.cols; i++)
  {
    for (int j = 0; j < edge.rows; j++)
    {
      if (edge.at<unsigned char>(j, i) > 0)
      {
        edge_points->points.emplace_back(static_cast<float>(i), static_cast<float>(j), 0);
      }
    }
  }

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(config_.octree_resolution);
  octree.setInputCloud(edge_points);
  octree.addPointsFromInputCloud();

  std::vector<Disc> covering_discs;
  for (int i = 0; i < inner_region.cols; i++)
  {
    for (int j = 0; j < inner_region.rows; j++)
    {
      if (inner_region.at<unsigned char>(j, i) > 0)
      {
        pcl::PointXYZ search_point(static_cast<float>(i), static_cast<float>(j), 0);
        std::vector<int> point_indices;
        std::vector<float> point_distance;
        if (octree.nearestKSearch(search_point, 1, point_indices, point_distance) > 0)
        {
          covering_discs.emplace_back(cv::Point2i {i, j}, std::sqrt(point_distance[0]));
        }
      }
    }
  }
  std::sort(covering_discs.begin(), covering_discs.end(), [](Disc a, Disc b) { return a.radius > b.radius; });

  std::vector<Disc> chosen_discs;

  auto overlap = [this](Disc disc, std::vector<Disc> chosen) {
    for (auto d : chosen)
    {
      double threshold = std::max(disc.radius, d.radius * (1 + config_.fudge_factor) - disc.radius);
      if (d.radius - disc.radius < disc.radius * config_.same_size_factor)
      {
        threshold = d.radius * config_.size_fudge_factor;
      }

      if (distance(d.center, disc.center) < threshold)
      {
        return true;
      }
    }
    return false;
  };
  // n^2
  for (auto disc : covering_discs)
  {
    if (disc.radius <= config_.disc_min_radius)
    {
      break;
    }
    if (!(overlap(disc, chosen_discs)))
    {
      chosen_discs.push_back(disc);
    }
  }

  if (config_.print_debug)
  {
    std::cout << "Number of discs: " << chosen_discs.size() << std::endl;
  }

  if (config_.save_covered_image)
  {
    for (auto disc : chosen_discs)
    {
      cv::circle(inner_region, disc.center, disc.radius, cv::Scalar(100), 1);
    }
    std::string saved_image_path(config_.save_folder);
    saved_image_path.append("/inner_region_covered.png");
    cv::imwrite(saved_image_path, inner_region);
  }
  return chosen_discs;
}

std::vector<Edge>
ClassifyRegion::buildGraph(std::vector<Disc> discs, cv::Mat inner_region)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_points(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto d : discs)
  {
    center_points->points.emplace_back(static_cast<float>(d.center.x), static_cast<float>(d.center.y), 0.0F);
  }
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(config_.octree_resolution);
  octree.setInputCloud(center_points);
  octree.addPointsFromInputCloud();

  std::vector<Edge> proposed_edges;
  auto intersect = [this](Disc p, Disc q) {
    auto dist = distance(p.center, q.center);
    return dist < (p.radius + q.radius) * config_.fudge_intersect_factor;
  };

  auto close = [this, discs](Edge e, std::vector<Edge> v) {
    for (auto g : v)
    {
      if (cosAngle(discs, e, g) > config_.cosine_threshold)
      {
        return true;
      }
    }
    return false;
  };

  int index = 0;
  for (auto d : discs)
  {
    pcl::PointXYZ search_point(d.center.x, d.center.y, 0.0F);
    std::vector<int> point_indices;
    std::vector<float> point_distance;
    std::vector<Edge> edge_from_disc;
    if (octree.nearestKSearch(search_point, config_.max_neighbors, point_indices, point_distance) > 0)
    {
      for (auto ind : point_indices)
      {
        if (ind == index)
        {
          continue;
        }
        if (intersect(d, discs[ind]) && !close({index, ind}, edge_from_disc))
        {
          edge_from_disc.emplace_back(index, ind);
        }
      }
    }
    proposed_edges.insert(proposed_edges.end(), edge_from_disc.begin(), edge_from_disc.end());
    index++;
  }

  if (config_.print_debug)
  {
    std::cout << "Number of edges: " << proposed_edges.size() << std::endl;
  }

  if (config_.save_proposed_edge_image)
  {
    for (auto e : proposed_edges)
    {
      cv::line(inner_region, discs[e.from].center, discs[e.to].center, cv::Scalar(150), 1, cv::LINE_8);
    }
    std::string saved_image_path(config_.save_folder);
    saved_image_path.append("/edge_proposal.png");
    cv::imwrite(saved_image_path, inner_region);
  }
  return proposed_edges;
}

cv::Mat
ClassifyRegion::markInnerRegion(std::string image_path)
{
  auto image = cv::imread(image_path, cv::IMREAD_UNCHANGED);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), config_.resize_ratio, config_.resize_ratio, cv::INTER_AREA);

  const auto resized_resolution = config_.original_resolution / config_.resize_ratio;
  const auto skip_distance_image = config_.skip_distance / resized_resolution;

  auto path_points =
      parseBagPath(config_.bag_path, resized.size(), resized_resolution, skip_distance_image, config_.topic_name);

  pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_points(new pcl::PointCloud<pcl::PointXYZ>);

  cv::Mat inner_candidates(resized.size(), CV_8U, cv::Scalar(0));
  const auto region_size_image = config_.candidate_region_size / resized_resolution;
  auto markInnerCandidates = [inner_candidates, region_size_image](cv::Point2i center) {
    cv::circle(inner_candidates, center, region_size_image, cv::Scalar(255), -1);
  };

  for (auto index : path_points)
  {
    trajectory_points->points.emplace_back(static_cast<float>(index.x), static_cast<float>(index.y), 0.0F);
    markInnerCandidates(index);
  }

  PointChecker point_checker(resized, config_.edge_threshold);
  std::cout << "Classifying" << std::endl;
  auto inner_region = classifyInnerRegion(trajectory_points, inner_candidates, point_checker);

  if (config_.save_inner_region_original)
  {
    cv::Mat inner_region_original;
    cv::resize(inner_region,
               inner_region_original,
               cv::Size(),
               1. / config_.resize_ratio,
               1. / config_.resize_ratio,
               cv::INTER_LINEAR);
    std::string saved_image_path(config_.save_folder);
    saved_image_path.append("/inner_region_original.png");
    cv::imwrite(saved_image_path, inner_region_original);
  }

  if (config_.save_inner_region)
  {
    std::string saved_image_path(config_.save_folder);
    saved_image_path.append("/inner_region.png");
    cv::imwrite(saved_image_path, inner_region);
  }

  return inner_region;
}
