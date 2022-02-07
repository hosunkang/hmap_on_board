#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(cloud_filtered);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
  pub.publish(output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_pc");
  ros::NodeHandle nh;
  ros::Subscriber sub=nh.subscribe("/camera/depth/color/points", 1, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("output",1);
  ros::spin();
}