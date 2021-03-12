#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <base/Angle.h>

void AngleCallback(const base::Angle::ConstPtr& msg);
tf2_ros::TransformBroadcaster* TF;
geometry_msgs::TransformStamped TFAngleMsg;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "angle2tf");
    ros::NodeHandle Nh;
    TF = new tf2_ros::TransformBroadcaster();
    ros::Subscriber AngleSubs=Nh.subscribe("/sensors/bodyAngle", 1, &AngleCallback);

    ros::spin();

    return 0;
}

void AngleCallback(const base::Angle::ConstPtr& msg)
{  
    tf2::Quaternion q; 
    TFAngleMsg.header.seq=msg->header.seq;
    TFAngleMsg.child_frame_id="jointRear";
    TFAngleMsg.header.frame_id="jointFront";
    TFAngleMsg.header.stamp=ros::Time::now();


    q.setRPY(0,0,msg->angle);
    TFAngleMsg.transform.translation.x=0;
    TFAngleMsg.transform.translation.y=0;
    TFAngleMsg.transform.translation.z=0;

    TFAngleMsg.transform.rotation.x=q.x();
    TFAngleMsg.transform.rotation.y=q.y();
    TFAngleMsg.transform.rotation.z=q.z();
    TFAngleMsg.transform.rotation.w=q.w();

    TF->sendTransform(TFAngleMsg);


}
