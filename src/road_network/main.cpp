#include <road_network/detect_route_graph.h>
#include <road_network/helper.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int
main(int argc, char** argv)
{
  auto config = readConfig(std::string(argv[1]));
  ClassifyRegion classifier(config);

  cv::Mat inner_region_image;
  if (config.mark_inner_region)
  {
    inner_region_image = classifier.markInnerRegion(config.image_path);
  }
  if (config.generate_graph)
  {
    if (inner_region_image.empty())
    {
      inner_region_image = cv::imread(config.inner_image_path, cv::IMREAD_GRAYSCALE);
    }
    auto covered = classifier.coverInnerRegion(inner_region_image);
    auto graph = classifier.buildGraph(covered, inner_region_image);
    saveGraph(covered, graph, config.save_graph_path);
  }
}
