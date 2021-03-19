#include <pathfollowing_node/pathfollowing.h>

PathFollowingControl::~PathFollowingControl(){
    
    //cleanup 
    delete _tfListener;
};

void PathFollowingControl::initialise(const ros::NodeHandle& nh, const float maxLinV, const float maxRotV,const float predTime,const std::string odomLink, const std::string baseLink)
{
    //initialise global variables
    PathFollowingControl::_nh=nh;
    PathFollowingControl::_maxLinV=maxLinV;
    PathFollowingControl::_maxRotV=maxRotV;
    PathFollowingControl::_prediction_time=predTime;
    PathFollowingControl::_odom_link =odomLink;
    PathFollowingControl::_base_link=baseLink;


    //initialise publisher
    PathFollowingControl::_futurePos_pub    = PathFollowingControl::_nh.advertise<geometry_msgs::PointStamped>("/futurePos", 1);
    PathFollowingControl::_targetPos_pub    = PathFollowingControl::_nh.advertise<geometry_msgs::PointStamped>("/targetPos", 1);
    PathFollowingControl::_cmdVel_pub       = PathFollowingControl::_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    //initialise subscriber
    PathFollowingControl::_path_sub = _nh.subscribe("/local_plan", 1, &PathFollowingControl::pathCallback,this);

    //initialise tf handler
    PathFollowingControl::_tfListener = new tf2_ros::TransformListener(_tfBuffer);

    PathFollowingControl::_isInit=true;
}

void PathFollowingControl::runControler(){
    if (_isInit==false){
        ROS_ERROR("PathFollowingControl::runControler Controler is not initialised.");
    }
    PathFollowingControl::_isFuturePosVal=PathFollowingControl::getTransform();

    if (PathFollowingControl::_isFuturePosVal){
        PathFollowingControl::_isFuturePosVal=PathFollowingControl::getFuturePos(_futurePos);
        PathFollowingControl::getClosestPoint(PathFollowingControl::_path);

    }
}

void PathFollowingControl::getClosestPoint(const nav_msgs::Path& path){
    float closest_dist=100000.0;
    float tmp_dist;
    tf2::Vector3 tmp_point;
    tf2::Vector3 p_start;
    tf2::Vector3 p_end;
    int path_size=path.poses.size();
    for (int i=0; i<path_size;i++){
        if (i<path_size-1){
            tf2::fromMsg(path.poses[i].pose.position,p_start);
            tf2::fromMsg(path.poses[i+1].pose.position,p_end);
            PathFollowingControl::applyScalarProjection(tmp_point,p_start,p_end);
        }
        else{
            tf2::fromMsg(path.poses[i].pose.position,tmp_point);
        }
        tmp_point.normalize();
        
        if(tmp_point.length()<closest_dist){
            PathFollowingControl::_targetPos=tmp_point;
        }
    }
}

//TODO: Check if distance to point to far 
void PathFollowingControl::pathCallback(const nav_msgs::Path& path){
    // PathFollowingControl::getClosestPoint(path);
    _path=path;
}

void PathFollowingControl::applyScalarProjection(tf2::Vector3& p_norm,tf2::Vector3& p_start, tf2::Vector3& p_end){
    
    tf2::Vector3 r_2_e=_futurePos-p_end; //cp vector between robot position and end of segment
    tf2::Vector3 line=p_end-p_start;    //cp linesegment
    line.normalize();
    p_norm=p_end+line*r_2_e.dot(line); //cp targetpoint on the linesegment

    
    float p_norm_len = p_norm.length2();
    float p_start_len = p_start.length2();
    float p_end_len = p_end.length2();

    if (p_norm_len<p_start_len && p_norm_len<p_end_len) p_norm=p_start;
    if (p_norm_len>p_start_len && p_norm_len>p_end_len) p_norm=p_end;
}

bool PathFollowingControl::getTransform(){
    try
    {
        tf2::fromMsg(PathFollowingControl::_tfBuffer.lookupTransform(PathFollowingControl::_base_link,PathFollowingControl::_odom_link,ros::Time(0),ros::Duration(0.2)).transform,PathFollowingControl::_odom_2_base);
        tf2::fromMsg(PathFollowingControl::_tfBuffer.lookupTransform(PathFollowingControl::_odom_link,PathFollowingControl::_base_link,ros::Time(0),ros::Duration(0.2)).transform,PathFollowingControl::_base_2_odom);
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("PathFollowingControl::getTransform(): %s",e.what());
        return false;
    }
    
}

bool PathFollowingControl::getFuturePos(tf2::Vector3& futurePos){
    try
    {
        futurePos.setZero();
        futurePos.setX(PathFollowingControl::_maxLinV * PathFollowingControl::_prediction_time);
        futurePos=PathFollowingControl::_base_2_odom*futurePos;
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("PathFollowingControl::getFuturePos(): %s",e.what());
        return false;
    }
}

bool PathFollowingControl::publish(bool visualize){
    try
    {
        if (visualize)
        {
            //Publish target position
            geometry_msgs::PointStamped target_point;
            target_point.point.x=_targetPos.getX();
            target_point.point.y=_targetPos.getY();
            target_point.header.stamp=ros::Time::now();
            target_point.header.frame_id=PathFollowingControl::_odom_link;
            PathFollowingControl::_targetPos_pub.publish(target_point);


            //Publish future Position
            geometry_msgs::PointStamped future_point;
            future_point.point.x=_futurePos.getX();
            future_point.point.y=_futurePos.getY();
            future_point.header.stamp=ros::Time::now();
            future_point.header.frame_id=PathFollowingControl::_odom_link;
            PathFollowingControl::_futurePos_pub.publish(future_point);
        }
            
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("PathFollowingControl::getTransform(): %s",e.what());
        return false;
    }
    
}