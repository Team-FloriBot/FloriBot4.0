#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include "../include/pointcloud_preprocessing.h"

using namespace std;
using namespace pcl; 

void pclPpc::pointcloudCalback (const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pclPpc::filterCon filterSetup = pclPpc::getParam();
  pcl::fromROSMsg(*input,*cloud);
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, filterSetup.rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, filterSetup.rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, filterSetup.gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, filterSetup.gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, filterSetup.bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, filterSetup.bMin)));
  

  
  // build the filter
  condrem.setCondition (color_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true); 

  // apply filter
  condrem.filter(*cloud_filtered);

  sensor_msgs::PointCloud2 cloud_filtered_msg;
  pcl::toROSMsg(*cloud_filtered,cloud_filtered_msg);
  // pcl_conversions::fromPCL(cloud_filtered,cloud_filtered_msg)
  
  pclPpc::pub.publish(cloud_filtered_msg);
  return ;
} 

pclPpc::filterCon pclPpc::getParam(){

  pclPpc::filterCon filterCon;
  ros::param::get("cloudPreprocessing/rMax",filterCon.rMax);
  ros::param::get("cloudPreprocessing/rMin",filterCon.rMin);
  ros::param::get("cloudPreprocessing/gMax",filterCon.gMax);
  ros::param::get("cloudPreprocessing/gMin",filterCon.gMin);
  ros::param::get("cloudPreprocessing/bMax",filterCon.bMax);
  ros::param::get("cloudPreprocessing/bMin",filterCon.bMin);
  
  ROS_INFO("%i", filterCon.gMax);
  return (filterCon);
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,pclPpc::nodeName);  
  ros::NodeHandle n;
  pclPpc::pub = n.advertise<sensor_msgs::PointCloud2>("cloud_out", 1000);
  ros::Subscriber sub =n.subscribe("cloud_in",1000,pclPpc::pointcloudCalback);
  ros::spin();
  return (0);
}