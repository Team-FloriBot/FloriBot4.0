#include <base_node/base_publisher.h>

KinematicsPublisher::KinematicsPublisher(ros::NodeHandle* pNh, kinematics::coordinate Base)
{
    seq_=0;
    pNh_=pNh;
    getParam();
    Angle_=0;
    Drive_.setParam(FrontLength_, RearLength_, AxesLength_, WheelDiameter_, Base);
    createPublisherSubscriber();
    CmdVelTimer_=pNh_->createTimer(ros::Duration(0.1), &KinematicsPublisher::PublishSpeed, this);
}
KinematicsPublisher::~KinematicsPublisher(){};

void KinematicsPublisher::PublishSpeed(const ros::TimerEvent& e)
{
    base::Wheels tmp;

    tmp.header.stamp=ros::Time::now();
    tmp.header.seq=seq_++;
    tmp.FrontLeft=Speedmsg_.FrontLeft;
    tmp.FrontRight=Speedmsg_.FrontRight;
    tmp.RearLeft=Speedmsg_.RearLeft;
    tmp.RearRight=Speedmsg_.RearRight;


    SpeedPublisher_.publish(Speedmsg_);

    Speedmsg_.FrontLeft=0;
    Speedmsg_.FrontRight=0;    
    Speedmsg_.RearRight=0;
    Speedmsg_.RearLeft=0;
}

void KinematicsPublisher::getParam()
{
    pNh_->param<double>("/"+ros::this_node::getName()+"/FrontLength", FrontLength_, 0.4);
    pNh_->param<double>("/"+ros::this_node::getName()+"/RearLength", RearLength_, 0.4);
    pNh_->param<double>("/"+ros::this_node::getName()+"/AxesLength", AxesLength_, 0.4);
    pNh_->param<double>("/"+ros::this_node::getName()+"/WheelDiameter", WheelDiameter_, 0.4);
}

void KinematicsPublisher::createPublisherSubscriber()
{
    OdometryPublisher_=pNh_->advertise<nav_msgs::Odometry>("/odom", 1);
    SpeedPublisher_=pNh_->advertise<base::Wheels>("engine/targetSpeed", 1);

    AngleSubscriber_=pNh_->subscribe("sensors/bodyAngle", 1, &KinematicsPublisher::AngleCallback, this);
    CmdVelSubscriber_=pNh_->subscribe("cmd_vel", 1, &KinematicsPublisher::CmdVelCallback, this);
    SpeedSubscriber_=pNh_->subscribe("engine/actualSpeed", 1, &KinematicsPublisher::SpeedCallback, this);    
}

void KinematicsPublisher::AngleCallback(const base::Angle::ConstPtr& msg)
{
    geometry_msgs::TransformStamped TFMsg;
    
    Angle_=msg->Angle;
    
    tf2::Quaternion q;
    q.setRPY(0,0,Angle_);

    TFMsg.child_frame_id="jointRear";
    TFMsg.header.frame_id="jointFront";
    TFMsg.header.seq=msg->header.seq;
    TFMsg.header.stamp=msg->header.stamp;

    TFMsg.transform.rotation.w=q.getW();
    TFMsg.transform.rotation.x=q.getX();
    TFMsg.transform.rotation.y=q.getY();
    TFMsg.transform.rotation.z=q.getZ();

    TFMsg.transform.translation.x=0;
    TFMsg.transform.translation.y=0;
    TFMsg.transform.translation.z=0;

    TFBroadaster_.sendTransform(TFMsg);
}

void KinematicsPublisher::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    kinematics::articulatedWheelSpeed Wheelspeed;

    Wheelspeed=Drive_.inverseKinematics(*msg);

    Speedmsg_.FrontLeft=Wheelspeed.Front.leftWheel;
    Speedmsg_.FrontRight=Wheelspeed.Front.rightWheel;

    Speedmsg_.RearLeft=Wheelspeed.Rear.leftWheel;
    Speedmsg_.RearRight=Wheelspeed.Rear.rightWheel;
}

void KinematicsPublisher::SpeedCallback(const base::Wheels::ConstPtr &msg)
{
    kinematics::articulatedWheelSpeed ActualSpeed;
    geometry_msgs::Pose2D ActualPose;
    geometry_msgs::TransformStamped Transform;
    nav_msgs::Odometry OdomMsg;
    tf2::Quaternion q;

    ActualSpeed.Front.leftWheel=msg->FrontLeft;
    ActualSpeed.Front.rightWheel=msg->FrontRight;
    ActualSpeed.Rear.leftWheel=msg->RearLeft;
    ActualSpeed.Rear.rightWheel=msg->RearRight;

    ActualPose=Drive_.forwardKinematics(ActualSpeed, msg->header.stamp );

     
    //Front Msg

    q.setRPY(0, 0, ActualPose.theta);

    //TF Msg
    Transform.child_frame_id="odom";
    Transform.header.frame_id="map";
    Transform.header.seq=msg->header.seq;
    Transform.header.stamp=msg->header.stamp;

    Transform.transform.translation.x=ActualPose.x;
    Transform.transform.translation.y=ActualPose.y;
    Transform.transform.translation.z=WheelDiameter_/2;

    Transform.transform.rotation.w=q.getW();
    Transform.transform.rotation.x=q.getX();
    Transform.transform.rotation.y=q.getY();
    Transform.transform.rotation.z=q.getZ();
   
    //ToDo: Add Covariance
    //Odom Msg
    OdomMsg.child_frame_id="odom";
    OdomMsg.header.frame_id="map";
    OdomMsg.header.seq=msg->header.seq;
    OdomMsg.header.stamp=msg->header.stamp;

    OdomMsg.pose.pose.orientation.w=q.getW();
    OdomMsg.pose.pose.orientation.x=q.getX();
    OdomMsg.pose.pose.orientation.y=q.getY();
    OdomMsg.pose.pose.orientation.z=q.getZ();

    OdomMsg.pose.pose.position.x=ActualPose.x;
    OdomMsg.pose.pose.position.y=ActualPose.y;
    OdomMsg.pose.pose.position.z=WheelDiameter_/2;


    OdomMsg.twist.twist=Drive_.getSpeed();

    //publish
    OdometryPublisher_.publish(OdomMsg);

    TFBroadaster_.sendTransform(Transform);

}