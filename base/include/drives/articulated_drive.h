#ifndef ARTICULATED_DRIVE_H
#define ARTICULATED_DRIVE_H

#include "drives/differential_drive.h"
#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>

namespace kinematics

{

struct articulatedWheelSpeed
{
    DifferentialWheelSpeed Front, Rear;
};

enum coordinate
{
    Front,
    Rear,
    JointFront,
    JointRear
};

class ArticulatedDrive
{
    public:
    ArticulatedDrive();
    ArticulatedDrive(double axesLength, double wheelDiameter, double frontlength, double rearlength, coordinate Base);
    ~ArticulatedDrive();

    articulatedWheelSpeed inverseKinematics(geometry_msgs::Twist cmdVelMsg, double Theta);
    articulatedWheelSpeed inverseKinematics(geometry_msgs::Twist cmdVelMsg, geometry_msgs::TransformStamped Front2Joint, 
            geometry_msgs::TransformStamped Joint2Rear);

    //Todo: Transform in baseframe
    geometry_msgs::Pose2D forwardKinematics(articulatedWheelSpeed WheelSpeed, ros::Time Timestamp);
    geometry_msgs::Pose2D estimateActualPose();
    geometry_msgs::Pose2D getActualPose(coordinate Frame);
    geometry_msgs::Twist getSpeed();

    void setParam(double FrontLength, double RearLength, double AxesLength, double WheelDiameter, coordinate Base);
    private:
    
    kinematics::coordinate Base_;
    kinematics::differentialDrive frontDrive_, rearDrive_;
    double frontlength_, rearlength_;
};
}
#endif