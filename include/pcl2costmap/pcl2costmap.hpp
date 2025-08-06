#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

namespace pcl2costmap {


class Pcl2Costmap : public rclcpp::Node {

 public:

  Pcl2Costmap();
  
  void setup();
  void voxel();
  void passthrough();
  void make_xy2d();
  
  void write_map_yaml();
  
  private:
  std::string map_yaml_path;
  std::string path_of_pcd;
  float voxel_resolution;
  float ground_thresh;
  float sky_thresh;
  
  bool use_costmap;
  float costmap_threshold;
  std::string pgm_name;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map;

  float origin_x, origin_y;
  int map_width, map_height;
  float map_resolution;

};

}
