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
        ros::Publisher                      _targetPos_pub;

        // Subscriber
        ros::Subscriber                     _odom_sub;
        ros::Subscriber                     _path_sub;

        //transformmessages
        tf2_ros::Buffer                     _tfBuffer;
        tf2_ros::TransformListener*          _tfListener;
        tf2::Transform                      _odom_2_base;
        tf2::Transform                      _base_2_odom;

        float                               _maxLinV;
        float                               _maxRotV;
        float                               _prediction_time;

        //frame
        std::string                         _odom_link;
        std::string                         _base_link;

        tf2::Vector3                        _futurePos;
        tf2::Vector3                        _actPos;
        tf2::Vector3                        _targetPos;
        nav_msgs::Path                      _path;

        bool getFuturePos(tf2::Vector3& futurePos);
        bool getTransform();
        void applyScalarProjection(tf2::Vector3& p_norm, tf2::Vector3& p_start, tf2::Vector3& p_end);
        void getClosestPoint(const nav_msgs::Path& path);

        //callbackfunktion
        void odomCallback(const nav_msgs::Odometry& odom);
        void pathCallback(const nav_msgs::Path& path);

    public:
        ~PathFollowingControl();
        void initialise(const ros::NodeHandle& nh ,const float maxLinV, const float maxRotV,const float predTime,const std::string odomLink, const std::string baseLink);
        bool publish(bool visualize);
        void runControler();
};
