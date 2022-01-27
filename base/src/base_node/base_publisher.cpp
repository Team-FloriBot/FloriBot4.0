#include <base_node/base_publisher.h>

KinematicsPublisher::KinematicsPublisher(ros::NodeHandle* pNh)
{
    seq_=0;
    pNh_=pNh;
    getParam();
    // initial Base (leading carriage) is set to coordinate::Front, 
    // it is switched automatically when driving backwards/ forwards
    Drive_.setParam(AxesLength_, WheelDiameter_, kinematics::coordinate::Front);
    createPublisherSubscriber();
    CmdVelTimer_=pNh_->createTimer(ros::Duration(1.0/PubFrequency_), &KinematicsPublisher::PublishSpeed, this);
}
KinematicsPublisher::~KinematicsPublisher(){};

void KinematicsPublisher::PublishSpeed(const ros::TimerEvent& e)
{
    base::Wheels tmp;
    tmp.header.stamp=ros::Time::now();
    tmp.header.seq=seq_++;
    if (tmp.header.stamp - Speedmsg_.header.stamp < ros::Duration(StopTimeout_)){
        tmp.frontLeft=Speedmsg_.frontLeft;
        tmp.frontRight=Speedmsg_.frontRight;
        tmp.rearLeft=Speedmsg_.rearLeft;
        tmp.rearRight=Speedmsg_.rearRight;
    }
    else{
        tmp.frontLeft=0;
        tmp.frontRight=0;    
        tmp.rearRight=0;
        tmp.rearLeft=0;
    }
    SpeedPublisher_.publish(tmp);
}

void KinematicsPublisher::getParam()
{
    pNh_->getParam("/"+ros::this_node::getName()+"/axesLength", this->AxesLength_);
    pNh_->getParam("/"+ros::this_node::getName()+"/wheelDiameter", this->WheelDiameter_);
    pNh_->getParam("/"+ros::this_node::getName()+"/pubFrequency", this->PubFrequency_);
    pNh_->getParam("/"+ros::this_node::getName()+"/stopTimeout", this->StopTimeout_);
}

void KinematicsPublisher::createPublisherSubscriber()
{
    ResetOdometryService_=pNh_->advertiseService("/reset_odom", &KinematicsPublisher::ResetOdometryCallback, this);

    OdometryPublisher_=pNh_->advertise<nav_msgs::Odometry>("/odom", 1);
    SpeedPublisher_=pNh_->advertise<base::Wheels>("/engine/targetSpeed", 1);

    CmdVelSubscriber_=pNh_->subscribe("/cmd_vel", 1, &KinematicsPublisher::CmdVelCallback, this);
    SpeedSubscriber_=pNh_->subscribe("/engine/actualSpeed", 1, &KinematicsPublisher::SpeedCallback, this);    
}

bool KinematicsPublisher::ResetOdometryCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    Drive_.frontDrive_.reset();
    Drive_.rearDrive_.reset();
    ROS_WARN("Service called. Reset robot odometry.");
    return true;
}

void KinematicsPublisher::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    kinematics::articulatedWheelSpeed Wheelspeed;

    Wheelspeed=Drive_.inverseKinematics(*msg);

    Speedmsg_.header.stamp=ros::Time::now();
    Speedmsg_.frontLeft=Wheelspeed.Front.leftWheel;
    Speedmsg_.frontRight=Wheelspeed.Front.rightWheel;

    Speedmsg_.rearLeft=Wheelspeed.Rear.leftWheel;
    Speedmsg_.rearRight=Wheelspeed.Rear.rightWheel;
}

void KinematicsPublisher::SpeedCallback(const base::Wheels::ConstPtr &msg)
{
    kinematics::articulatedWheelSpeed ActualSpeed;
    geometry_msgs::Pose2D OdomPose;
    geometry_msgs::TransformStamped Transform;
    nav_msgs::Odometry OdomMsg;
    tf2::Quaternion q;

    ActualSpeed.Front.leftWheel=msg->frontLeft;
    ActualSpeed.Front.rightWheel=msg->frontRight;
    ActualSpeed.Rear.leftWheel=msg->rearLeft;
    ActualSpeed.Rear.rightWheel=msg->rearRight;

    OdomPose=Drive_.forwardKinematics(ActualSpeed, msg->header.stamp );

     
    //Front Msg
    q.setRPY(0, 0, OdomPose.theta);

    //TF Msg
    Transform.child_frame_id="base_link";
    Transform.header.frame_id="odom";
    Transform.header.seq=msg->header.seq;
    Transform.header.stamp=msg->header.stamp;

    Transform.transform.translation.x=OdomPose.x;
    Transform.transform.translation.y=OdomPose.y;
    Transform.transform.translation.z=WheelDiameter_/2;

    Transform.transform.rotation.w=q.getW();
    Transform.transform.rotation.x=q.getX();
    Transform.transform.rotation.y=q.getY();
    Transform.transform.rotation.z=q.getZ();
   
    //ToDo: Add Covariance
    //Odom Msg
    OdomMsg.child_frame_id="base_link";
    OdomMsg.header.frame_id="odom";
    OdomMsg.header.seq=msg->header.seq;
    OdomMsg.header.stamp=msg->header.stamp;

    OdomMsg.pose.pose.orientation.w=q.getW();
    OdomMsg.pose.pose.orientation.x=q.getX();
    OdomMsg.pose.pose.orientation.y=q.getY();
    OdomMsg.pose.pose.orientation.z=q.getZ();

    OdomMsg.pose.pose.position.x=OdomPose.x;
    OdomMsg.pose.pose.position.y=OdomPose.y;
    OdomMsg.pose.pose.position.z=WheelDiameter_/2;


    //According to http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom the speed has to be in the child_frame
    //in our case base_link which means the robot itself
    //But now odom links to nav_base which changes between axesFront (forwards driving) and axesRear (backards driving)

    OdomMsg.twist.twist=Drive_.getSpeed();    

    //publish
    OdometryPublisher_.publish(OdomMsg);

    TFBroadaster_.sendTransform(Transform);

}