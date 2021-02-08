#include <pathfollowing_node/pathfollowing.h>

PathFollowingControl::~PathFollowingControl(){};

void PathFollowingControl::initialise(const ros::NodeHandle& nh, const float maxLinV, const float maxRotV,const float timejump)
{
    //initialise global variables
    PathFollowingControl::_nh=nh;
    PathFollowingControl::_maxLinV=maxLinV;
    PathFollowingControl::_maxRotV=maxRotV;
    PathFollowingControl::_timejump=timejump;

    //initialise publisher
    PathFollowingControl::_futurePos_pub    = PathFollowingControl::_nh.advertise<geometry_msgs::PointStamped>("/futurePos", 1);
    PathFollowingControl::_cmdVel_pub       = PathFollowingControl::_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //initialise subscriber
    PathFollowingControl::_odom_sub = _nh.subscribe("/odom", 1, &PathFollowingControl::odomCallback,this);

    //initialise tf handler

}

void PathFollowingControl::odomCallback(const nav_msgs::Odometry& odom){
    PathFollowingControl::getTransform();
    geometry_msgs::PointStamped futurePos;
    geometry_msgs::PointStamped actPos;
    actPos.header=odom.header;
    actPos.point=odom.pose.pose.position;
    PathFollowingControl::getFuturePos(futurePos,actPos);
}

bool PathFollowingControl::getTransform(){
    try
    {
        PathFollowingControl::_odom_2_base = PathFollowingControl::_tfBuffer.lookupTransform("base_link","odom",ros::Time(0));
        PathFollowingControl::_base_2_odom = PathFollowingControl::_tfBuffer.lookupTransform("odom","base_link",ros::Time(0));
        return false;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    
}

bool PathFollowingControl::getFuturePos(geometry_msgs::PointStamped& futurePos, geometry_msgs::PointStamped& actPos){
    try
    {
        tf2::doTransform(actPos,futurePos,PathFollowingControl::_odom_2_base);
        futurePos.point.x+=PathFollowingControl::_maxLinV * PathFollowingControl::_timejump;
        tf2::doTransform(futurePos,futurePos,PathFollowingControl::_base_2_odom);
        PathFollowingControl::_futurePos_pub.publish(futurePos);
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}