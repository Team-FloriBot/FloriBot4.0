#include <pathfollowing_node/pathfollowing.h>

PathFollowingControl::~PathFollowingControl(){
    
    //cleanup 
    _tfListener->~TransformListener();
    delete _tfListener;
};

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
    PathFollowingControl::_odom_sub = _nh.subscribe("/local_plan", 1, &PathFollowingControl::PathCallback,this);

    //initialise tf handler
    PathFollowingControl::_tfListener = new tf2_ros::TransformListener(_tfBuffer);

}

void PathFollowingControl::odomCallback(const nav_msgs::Odometry& odom){
    PathFollowingControl::_isFuturePosVal=PathFollowingControl::getTransform();

    if (PathFollowingControl::_isFuturePosVal){
        _actPos.header=odom.header;
        _actPos.point=odom.pose.pose.position;
        PathFollowingControl::_isFuturePosVal=PathFollowingControl::getFuturePos(_futurePos,_actPos);
    }
}

//TODO: Check if distance to point to far 
void PathFollowingControl::PathCallback(const nav_msgs::Path& path){
    if(PathFollowingControl::_isFuturePosVal){
        for (int i=0; i<path.poses.size();i++){

            
        }
    }
}

void PathFollowingControl::applyScalarProjection(geometry_msgs::Point& p_norm,geometry_msgs::Point& p_start, geometry_msgs::Point& p_end){
    // std::vector<double> _ap={   PathFollowingControl::_futurePos.point.x-p_end.x,
    //                             PathFollowingControl::_futurePos.point.y-p_end.y,
    //                             PathFollowingControl::_futurePos.point.z-p_end.z};
    // double _ap[3]={ PathFollowingControl::_futurePos.point.x-p_end.x,
    //                 PathFollowingControl::_futurePos.point.y-p_end.y,
    //                 PathFollowingControl::_futurePos.point.z-p_end.z};
    // std::vector<double> _ab={   p_end.x-p_start.x,
    //                             p_end.y-p_start.y,
    //                             p_end.z-p_start.z};
    // _ab=_ab/std::sqrt(std::pow(_ab[0],2)+std::pow(_ab[1],2)+std::pow(_ab[2],2));    


    // std::vector<double> _p_norm=_p_end-_p_start;

    // std::
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