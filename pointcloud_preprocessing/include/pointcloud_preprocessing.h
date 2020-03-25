#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

using namespace std;
using namespace pcl; 


namespace pclPpc{
    struct filterCon
    {
        int rMax = 255;
        int rMin = 150;
        int gMax = 100;
        int gMin = 0;
        int bMax = 100;
        int bMin = 0;
    };

    void pointcloudCalback(const sensor_msgs::PointCloud2::ConstPtr& input);
    filterCon getParam();

    // build the condition filter
    
    std::string nodeName="pointcloudPreprocessingNode";

    ros::Publisher pub;
   
}

