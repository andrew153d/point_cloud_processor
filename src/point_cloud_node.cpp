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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <iostream>

using namespace std;

#define OBSTACLE_FILTER_MIN 0.1
#define OBSTACLE_FILTER_MAX 1.5


ros::Publisher pub;
ros::Publisher cost_map_pub;
ros::Publisher obstacle_map;
ros::Publisher obstaclePub;

class block
{
public:
  float x, y, z;
  float count;
  block *parent;
  std::vector<block*> kids;

  bool isParent(){
    return !kids.empty();
  }
};
float averagePointDensity; //The average number of points in a block

std::vector<block> blockList; // a list of all blocks containing the 

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Convert the ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromROSMsg(*msg, cloud);
  
  // Remove NaN points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, indices);

  float angle = 10.0; // Rotate 45 degrees around the z-axis
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0, 1; // Shift the point cloud up by 0.5 meters
  transform.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitY()));
  // Apply the rotation to the cloud
  
  pcl::transformPointCloud(cloud, cloud, transform);

  // Create a PassThrough filter to filter the cetloud bween z=0 and z=2
  pcl::PassThrough<pcl::PointXYZRGBA> pass2;
  pcl::PointCloud<pcl::PointXYZRGBA> obstacle_cloud;
  pass2.setInputCloud(cloud.makeShared());
  pass2.setFilterFieldName("z");
  pass2.setFilterLimits(OBSTACLE_FILTER_MIN, OBSTACLE_FILTER_MAX);
  pass2.filter(obstacle_cloud);
  
  sensor_msgs::PointCloud2 obstacle_cloud_msg;
  pcl::toROSMsg(obstacle_cloud, obstacle_cloud_msg);
  obstacle_cloud_msg.header = msg->header;
  obstaclePub.publish(obstacle_cloud_msg);
  
  

  // Filter points based on height
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.3, 0.3);
  // pass.setFilterLimitsNegative (true);
  pass.filter(cloud);

  // pass.setInputCloud(cloud.makeShared());
  // pass.setFilterFieldName("x");
  // pass.setFilterLimits(0, 3);
  //  pass.setFilterLimitsNegative (true);
  // pass.filter(cloud);
  pcl::ConditionalRemoval<pcl::PointXYZRGBA> color_filter;

  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("r", pcl::ComparisonOps::GT, 150));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("g", pcl::ComparisonOps::GT, 150));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr blue_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("b", pcl::ComparisonOps::GT, 150));

  pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGBA>());
  color_cond->addComparison(red_condition);
  color_cond->addComparison(green_condition);
  color_cond->addComparison(blue_condition);

  // Build the filter
  color_filter.setInputCloud(cloud.makeShared());
  color_filter.setCondition(color_cond);
  color_filter.filter(cloud);

  // Convert the filtered point cloud to ROS message
  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(cloud, filtered_msg);
  filtered_msg.header = msg->header;
  pub.publish(filtered_msg);

  nav_msgs::OccupancyGrid map;
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "base_link";
  map.info.map_load_time = ros::Time::now();
  map.info.resolution = 0.1;         // set your desired resolution
  map.info.width = 50;               // set your desired width
  map.info.height = 50;              // set your desired height
  map.info.origin.position.x = 0;    // set your desired origin position
  map.info.origin.position.y = -2.5; // set your desired origin position
  map.info.origin.position.z = 0;    // set your desired origin position
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);
  for (int i = 0; i < map.data.size(); i++)
  {
    map.data[i] = -1;
  }
  for (int i = 0; i < 50; i++)
  {
    map.data[i] = i * 2;
  }
  for (auto point : cloud.points)
  {
    int px = (int)(point.x / 0.1);
    int py = ((int)((point.y + 2.5) / 0.1));
    int i = px + 50 * py;
    // printf("point %d, %d \n", px, py);
    if (i < 50 * 50 and i > 0)
    {
      map.data[i]++;
      if (map.data[i] > 100)
      {
        map.data[i] = 100;
      }
    }
  }

  for(int i = 0; i<map.data.size(); i++){
    map.data[i]= (map.data[i]>70)? 100 : 0;
  }
  
  cost_map_pub.publish(map);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_node");
  ros::NodeHandle nh;

  ros::Subscriber cloud_sub = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_callback);

  
  cost_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("filter/cost_map", 1);
  
  pub = nh.advertise<sensor_msgs::PointCloud2>("filter/output", 1);
  obstaclePub = nh.advertise<sensor_msgs::PointCloud2>("filter/obstacles", 1);
  
  ros::spin();
  return 0;
}
