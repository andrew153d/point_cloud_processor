#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/GridCells.h>
#include "sensor_msgs/Imu.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <cmath>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

ros::Publisher pub;
ros::Publisher grid_cells_pub;

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Convert the ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // Remove NaN points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, indices);

  // Filter points based on height
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-10, 0);
  // pass.setFilterLimitsNegative (true);
  pass.filter(cloud);
  uint32_t blue = (static_cast<uint32_t>(0) << 16) | (static_cast<uint32_t>(0) << 8) | static_cast<uint32_t>(255);
  for (auto point : cloud.points)
  {
    // std::cout << "hi" << (point.r + point.g + point.b) / 3 << std::endl;
    if ((point.r + point.g + point.b) / 3 < 250)
    {

      point.rgb = blue;
    }
  }

  // cloud.points.erase(std::remove_if(cloud.points.begin(), cloud.points.end(), [](const pcl::PointXYZRGBA &point)
  //                                  { return (point.r + point.g + point.b) / 3 < 100; }),
  //                   cloud.points.end());

  // Convert the filtered point cloud to ROS message
  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(cloud, filtered_msg);
  filtered_msg.header = msg->header;
  pub.publish(filtered_msg);

  nav_msgs::GridCells grid_cells;
  grid_cells.header.frame_id = "base_link";
  grid_cells.cell_width = 0.1;
  grid_cells.cell_height = 0.1;
  std::vector<geometry_msgs::Point> points;
  
  int width = 10;
  int height = 10;
  float resolution = 0.2;


  for (int i = 0; i < width*height; i++)
  {
    geometry_msgs::Point point;
    point.x = (i%width)*resolution;
    point.y = int(i/width)*resolution;
    point.z = 0;
    points.push_back(point);
  }

  grid_cells.cells = points;
  grid_cells_pub.publish(grid_cells);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_node");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  grid_cells_pub = nh.advertise<nav_msgs::GridCells>("grid_cells", 10);
  ros::Subscriber cloud_sub = nh.subscribe("/zed2/zed_node/point_clo1", 1, cloud_callback);

  ros::spin();
  return 0;
}
