#include "drives/articulated_drive.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

kinematics::ArticulatedDrive::ArticulatedDrive()
{}


kinematics::ArticulatedDrive::ArticulatedDrive(double axesLength, double wheelDiameter, double frontlength, double rearlength, coordinate Base):
        frontDrive_(axesLength, wheelDiameter), rearDrive_(axesLength, wheelDiameter), frontlength_(frontlength), rearlength_(rearlength), Base_(Base)
        {}


kinematics::ArticulatedDrive::~ArticulatedDrive() {}


kinematics::articulatedWheelSpeed kinematics::ArticulatedDrive::inverseKinematics(geometry_msgs::Twist cmdVelMsg, double Theta)
{
    articulatedWheelSpeed retVal;
    geometry_msgs::Twist FrontMsg, RearMsg;

    //Calculate Speeds for the two differential drives regarding the base frame
    switch (Base_)
    {
    //Front Speed is given
    case coordinate::Front:
        FrontMsg=cmdVelMsg;
        RearMsg.angular.z=(FrontMsg.linear.x*sin(Theta)-FrontMsg.angular.z*frontlength_*cos(Theta))/rearlength_;
        RearMsg.linear.x=FrontMsg.linear.x*cos(Theta)+FrontMsg.angular.z*frontlength_*sin(Theta);
        break;
        
    //RearSpeed is given
    case coordinate::Rear:
        RearMsg=cmdVelMsg;
        FrontMsg.angular.z=-(RearMsg.linear.x*sin(Theta)+RearMsg.angular.z*rearlength_*cos(Theta))/frontlength_;
        FrontMsg.linear.x=RearMsg.linear.x*cos(Theta)-RearMsg.angular.z*rearlength_*sin(Theta);
        break;

    //Do not calculate when any other Frame is given
    default:
        throw new std::runtime_error("Only Front and Rear Frames are allowed for inverse kinematics");
        return retVal;
    }

    retVal.Front=frontDrive_.inverseKinematics(FrontMsg);
    retVal.Rear=rearDrive_.inverseKinematics(RearMsg);

    return retVal;
}


//Todo Return all Frames
geometry_msgs::Pose2D kinematics::ArticulatedDrive::forwardKinematics(articulatedWheelSpeed WheelSpeed, ros::Time Timestamp)
{
    geometry_msgs::Pose2D FrontPose=frontDrive_.forwardKinematics(WheelSpeed.Front, Timestamp);
    geometry_msgs::Pose2D RearPose=rearDrive_.forwardKinematics(WheelSpeed.Rear, Timestamp);

    switch (Base_)
    {
        case coordinate::Front:
            return FrontPose;
            break;

        case coordinate::Rear:
            return RearPose;
            break;

        default:
            throw new std::runtime_error("Can not calculate forwardkinematics for given Frame");
    }
}


geometry_msgs::Pose2D kinematics::ArticulatedDrive::estimateActualPose()
{
    switch (Base_)
    {
    case coordinate::Front:
        return frontDrive_.estimateActualPose();
        break;

    case coordinate::Rear:
        return rearDrive_.estimateActualPose();
        break;

    default:
        throw new std::runtime_error("Can not estimate pose for given Frame");
    }
}

void kinematics::ArticulatedDrive::setParam(double FrontLength, double RearLength, double AxesLength, double WheelDiameter, coordinate Base)
{
    frontDrive_.setParam(AxesLength, WheelDiameter);
    rearDrive_.setParam(AxesLength, WheelDiameter);
    frontlength_=FrontLength;
    rearlength_=RearLength;
    Base_=Base;
}


geometry_msgs::Pose2D kinematics::ArticulatedDrive::getActualPose(coordinate Frame)
{
    switch (Base_)
    {
        case coordinate::Front:
            return frontDrive_.getActualPose();
            break;
        case coordinate::Rear:
            return rearDrive_.getActualPose();
            break;
        default:
            throw new std::runtime_error("Can not get Pose for given Frame");
    }
}

geometry_msgs::Twist kinematics::ArticulatedDrive::getSpeed()
{
    switch (Base_)
    {
        case coordinate::Front:
            return frontDrive_.getSpeed();
            break;

        case coordinate::Rear:
            return rearDrive_.getSpeed();
            break;

        default:
            throw new std::runtime_error("Can not get Speed for given Frame");
    }
}