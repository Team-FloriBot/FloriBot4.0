#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <vector>

geometry_msgs::Twist cmd_vel;


float getMaxAngularV(float linearV, float min_turn_angle);
geometry_msgs::Pose2D getPointerToPoint(geometry_msgs::Pose2D point, nav_msgs::Odometry odom);
int cmpManover();

int main(int argc, char **argv){
    ros::init(argc, argv,"pathfollowing");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);
    ros::Rate loop_rate(10);

    cmd_vel.linear.x=0;
    cmd_vel.angular.y=0;
    float min_turn_angle=0.7;

    while (ros::ok()){

        chatter_pub.publish(cmd_vel);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){


    return;
}

float getMaxAngularV(float linearV, float min_turn_angle){ //Simple computation of the maximal angular speed
    return linearV/min_turn_angle;
}

geometry_msgs::Pose2D getPointerToPoint(geometry_msgs::Pose2D point, nav_msgs::Odometry::ConstPtr& odom){
    geometry_msgs::Pose2D ptPoint;

    ptPoint.x= point.x-&odom.linear.x;//-cmd_vel.linear.x*std::cos(odom.angular.z);
    ptPoint.y= point.y-&odom.linear.y;//-cmd_vel.linear.x*std::cos(odom.angular.z);
}

int cmpManover(){

}






















