#include <functional>
#include <pcl2costmap/pcl2costmap.hpp>

namespace pcl2costmap {

Pcl2Costmap::Pcl2Costmap() : Node("pcl2costmap") {
  this->declare_parameter<std::string>("path_of_pcd", "/path/of/pcd");
  this->declare_parameter<double>("voxel_resolution", 0.0);
  this->declare_parameter<double>("ground_thresh", 0.0);
  this->declare_parameter<double>("sky_thresh", 0.0);
  this->declare_parameter<bool>("use_costmap", false);
  this->declare_parameter<double>("costmap_threshold", 0.0);
  this->declare_parameter<std::string>("pgm_name", "map");
}

void Pcl2Costmap::setup() {
  this->get_parameter("path_of_pcd", path_of_pcd);
  RCLCPP_INFO(this->get_logger(), "path_of_pcd: %s", path_of_pcd.c_str());
  this->get_parameter("voxel_resolution", voxel_resolution);
  RCLCPP_INFO(this->get_logger(), "voxel_resolution: %.2f", voxel_resolution);
  this->get_parameter("ground_thresh", ground_thresh);
  RCLCPP_INFO(this->get_logger(), "ground_thresh: %.2f", ground_thresh);  
  this->get_parameter("sky_thresh", sky_thresh);
  RCLCPP_INFO(this->get_logger(), "sky_thresh: %.2f", sky_thresh);
  this->get_parameter("use_costmap", use_costmap);
  RCLCPP_INFO(this->get_logger(), "use_costmap: %s",
              use_costmap ? "true" : "false");
  this->get_parameter("costmap_threshold", costmap_threshold);
  RCLCPP_INFO(this->get_logger(), "costmap_threshold: %.2f", costmap_threshold);
  this->get_parameter("pgm_name", pgm_name);
  RCLCPP_INFO(this->get_logger(), "pgm_name: %s", pgm_name.c_str());

  cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  voxel_map =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path_of_pcd, *cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s",
                 path_of_pcd.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu points from %s", cloud->size(),
              path_of_pcd.c_str());
}

void Pcl2Costmap::voxel() {
  this->voxel_map = this->cloud;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(voxel_map);
  voxel_filter.setLeafSize(voxel_resolution, voxel_resolution,
                           voxel_resolution);
  voxel_filter.filter(*voxel_map);
}

void Pcl2Costmap::make_xy2d() {
  if (voxel_map->empty()) {
    RCLCPP_WARN(this->get_logger(), "Voxel Cloud is empty.");
    return;
  }

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();

  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  float margin = 0.5;
  min_x -= margin;
  max_x += margin;
  min_y -= margin;
  max_y += margin;

  float resolution = voxel_resolution;  // 10cm/pixel
  int width = static_cast<int>((max_x - min_x) / resolution);
  int height = static_cast<int>((max_y - min_y) / resolution);

  RCLCPP_INFO(this->get_logger(), "Creating image: %d x %d", width, height);

  cv::Mat map_image =
      cv::Mat::zeros(height, width, CV_8UC1);  // (type : CV_8UC1)

  // Convert cloud point XY to image coordinates and mark the point
  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;

    int u = static_cast<int>((pt.x - min_x) / resolution);
    int v = static_cast<int>((pt.y - min_y) / resolution);

    //  Since OpenCV has (0,0) at the top-left, flip the Y-axis
    int row = height - 1 - v;
    int col = u;

    if (row >= 0 && row < height && col >= 0 && col < width) {
      if (pt.z < ground_thresh) {
        map_image.at<unsigned char>(row, col) = 127; 
      } 
      else if (pt.z> sky_thresh){
      continue;
      }
      else {
        map_image.at<unsigned char>(row, col) = 255;  // 흰색: 장애물
      }
    }
  }

  //contour
  cv::Mat erode_image =cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat dilate_image =cv::Mat::zeros(height, width, CV_8UC1);

  cv::erode(map_image, erode_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(erode_image, dilate_image, cv::Mat(), cv::Point(-1, -1), 1);

  // save
  cv::imwrite(pgm_name + ".png", dilate_image);
  RCLCPP_INFO(this->get_logger(), "Saved image as %s", pgm_name.c_str());
  
  
  RCLCPP_INFO(this->get_logger(), "origin_x: %f, origin_y: %f", min_x, min_y);
  RCLCPP_INFO(this->get_logger(), "map_resolution: %f", map_resolution);
  origin_x = min_x;
  origin_y = min_y;
  map_width = width;
  map_height = height;
  map_resolution = resolution;
}
}  // namespace pcl2costmap

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto pcd_inverter = std::make_shared<pcl2costmap::Pcl2Costmap>();
  if (rclcpp::ok()) {
    RCLCPP_INFO(pcd_inverter->get_logger(), "start");
    rclcpp::spin_some(pcd_inverter);
    RCLCPP_INFO(pcd_inverter->get_logger(), "setup");
    pcd_inverter->setup();
    // RCLCPP_INFO(pcd_inverter->get_logger(), "passthrough");
    // pcd_inverter->passthrough();
    RCLCPP_INFO(pcd_inverter->get_logger(), "voxel");
    pcd_inverter->voxel();
    RCLCPP_INFO(pcd_inverter->get_logger(), "make_xy2d");
    pcd_inverter->make_xy2d();
  }
  rclcpp::shutdown();
  return 0;
}
