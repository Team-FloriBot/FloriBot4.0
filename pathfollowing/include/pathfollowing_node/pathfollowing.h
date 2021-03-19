#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


   
class PathFollowingControl
{
    private:
        bool                                _isInit=false;
        bool                                _isFuturePosVal=false;
        ros::NodeHandle                     _nh;

        // Publisher
        ros::Publisher                      _cmdVel_pub;
        ros::Publisher                      _futurePos_pub;

        // Subscriber
        ros::Subscriber                     _odom_sub;
        ros::Subscriber                     _path_sub;

        //transformmessages
        tf2_ros::Buffer                     _tfBuffer;
        tf2_ros::TransformListener*          _tfListener;
        geometry_msgs::TransformStamped     _odom_2_base;
        geometry_msgs::TransformStamped     _base_2_odom;

        float                               _maxLinV;
        float                               _maxRotV;
        float                               _timejump;

        geometry_msgs::PointStamped         _futurePos;
        geometry_msgs::PointStamped         _actPos;

        bool getFuturePos(geometry_msgs::PointStamped& futurePos, geometry_msgs::PointStamped& actPos);
        bool getTransform();
        void applyScalarProjection(geometry_msgs::Point& p_norm, geometry_msgs::Point& p_start, geometry_msgs::Point& p_end);

        //callbackfunktion
        void odomCallback(const nav_msgs::Odometry& odom);
        void PathCallback(const nav_msgs::Path& path);

    public:
        void initialise(const ros::NodeHandle& nh ,const float maxLinV, const float maxRotV,const float timejump);
        ~PathFollowingControl();
};
