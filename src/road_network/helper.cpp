#include <fstream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/emittermanip.h>
#include <yaml-cpp/yaml.h>

#include <road_network/helper.h>

double
distance(cv::Point2i p, cv::Point2i q)
{
  return std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));
};

cv::Point2i
convertToIndex(const double x, const double y, const double resolution, const cv::Size image_size)
{
  const auto x_index = -y / resolution + image_size.height / 2.0;
  const auto y_index = -x / resolution + image_size.width / 2.0;
  return cv::Point2i(static_cast<int>(x_index), static_cast<int>(y_index));
}

RouteGraphConfig
readConfig(std::string config_name)
{
  RouteGraphConfig config;
  const auto yaml = YAML::LoadFile(config_name);
  config.image_path = yaml["image_path"].as<std::string>();
  config.bag_path = yaml["bag_path"].as<std::string>();
  config.inner_image_path = yaml["inner_image_path"].as<std::string>();
  config.topic_name = yaml["topic_name"].as<std::string>();
  config.original_resolution = yaml["original_resolution"].as<double>();
  config.resize_ratio = yaml["resize_ratio"].as<double>();
  config.candidate_region_size = yaml["candidate_region_size"].as<double>();
  config.skip_distance = yaml["skip_distance"].as<double>();
  config.octree_resolution = yaml["octree_resolution"].as<int>();
  config.edge_threshold = yaml["edge_threshold"].as<int>();
  config.save_folder = yaml["save_folder"].as<std::string>();

  config.print_debug = yaml["print_debug"].as<bool>();
  config.save_inner_region = yaml["save_inner_region"].as<bool>();
  config.save_inner_region_original = yaml["save_inner_region_original"].as<bool>();
  config.blur_kernel = yaml["blur_kernel"].as<int>();
  config.laplacian_kernel = yaml["laplacian_kernel"].as<int>();
  config.laplacian_scale = yaml["laplacian_scale"].as<int>();
  config.laplacian_delta = yaml["laplacian_delta"].as<int>();

  config.disc_min_radius = yaml["disc_min_radius"].as<double>();
  config.fudge_factor = yaml["fudge_factor"].as<double>();
  config.size_fudge_factor = yaml["size_fudge_factor"].as<double>();
  config.same_size_factor = yaml["same_size_factor"].as<double>();
  config.save_covered_image = yaml["save_covered_image"].as<bool>();
  config.max_neighbors = yaml["max_neighbors"].as<int>();
  config.fudge_intersect_factor = yaml["fudge_intersect_factor"].as<double>();
  config.cosine_threshold = yaml["cosine_threshold"].as<double>();
  config.save_proposed_edge_image = yaml["save_proposed_edge_image"].as<bool>();
  config.save_graph_path = yaml["save_graph_path"].as<std::string>();

  config.mark_inner_region = yaml["mark_inner_region"].as<bool>();
  config.generate_graph = yaml["generate_graph"].as<bool>();
  return config;
}

std::vector<cv::Point2i>
parseBagPath(std::string bag_path, cv::Size image_size, double resolution, double skip_distance, std::string topic)
{
  bool first_point = false;
  cv::Point2i last_point;

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::BagMode::Read);
  std::vector<std::string> topics;
  topics.push_back(topic);

  std::vector<cv::Point2i> path_points;
  for (rosbag::MessageInstance const message : rosbag::View(bag, rosbag::TopicQuery(topics)))
  {
    const auto position = message.instantiate<nav_msgs::Odometry>();
    const auto index =
        convertToIndex(position->pose.pose.position.x, position->pose.pose.position.y, resolution, image_size);
    if (!first_point)
    {
      last_point = index;
      first_point = true;
    }
    else if (std::abs(index.x - last_point.x) + std::abs(index.y - last_point.y) < skip_distance)
    {
      continue;
    }

    path_points.push_back(index);
    last_point = index;
  }
  return path_points;
}

double
cosAngle(std::vector<Disc> discs, Edge e, Edge f)
{
  auto vectorConnection = [discs](Edge e) {
    return cv::Point2i {discs[e.from].center.x - discs[e.to].center.x, discs[e.from].center.y - discs[e.to].center.y};
  };
  auto normalize = [](cv::Point2i vec) {
    double magnitude = distance(vec, cv::Point2i {0, 0});
    return cv::Point2d {vec.x / magnitude, vec.y / magnitude};
  };
  auto dotProduct = [](cv::Point2d p, cv::Point2d q) {
    return p.x * q.x + p.y * q.y;
  };

  auto e_con = normalize(vectorConnection(e));
  auto f_con = normalize(vectorConnection(f));
  return dotProduct(e_con, f_con);
}

void
saveGraph(std::vector<Disc> discs, std::vector<Edge> edges, std::string path)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "nodes" << YAML::Value;
  out << YAML::BeginSeq;
  for (int i = 0; i < discs.size(); i++)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << i;
    out << YAML::Key << "x" << YAML::Value << discs[i].center.x;
    out << YAML::Key << "y" << YAML::Value << discs[i].center.y;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "edges" << YAML::Value;
  out << YAML::BeginSeq;
  for (int i = 0; i < edges.size(); i++)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << i;
    out << YAML::Key << "vertices" << YAML::Value;
    out << YAML::BeginSeq;
    out << edges[i].from;
    out << edges[i].to;
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out.SetIndent(2);

  std::ofstream graph_file(path);
  graph_file << out.c_str();
}
