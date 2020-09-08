#ifndef KINEMATICS_PUBLISHER_H
#define KINEMATICS_PUBLISHER_H

#include "drives/articulated_drive.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "base/Angle.h"
#include <nav_msgs/Odometry.h>

#include "base/Wheels.h"

class KinematicsPublisher
{
public:
    KinematicsPublisher(ros::NodeHandle* pnh, kinematics::coordinate Base);
    ~KinematicsPublisher();

    void getParam();
    void createPublisherSubscriber();
    void PublishSpeed();

private:
    
    void AngleCallback(const base::Angle::ConstPtr& msg);
    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void SpeedCallback(const base::Wheels::ConstPtr& msg);

    kinematics::ArticulatedDrive Drive_;
    base::Wheels Speedmsg_;
    ros::NodeHandle* pNh_;
    ros::Timer CmdVelTimer_;
    ros::Publisher SpeedPublisher_, OdometryPublisher_;
    ros::Subscriber AngleSubscriber_, CmdVelSubscriber_, SpeedSubscriber_;
    tf2_ros::TransformBroadcaster TFBroadaster_;
    double Angle_, FrontLength_, RearLength_, AxesLength_, WheelDiameter_;
    unsigned int seq_;
    
};


#endif