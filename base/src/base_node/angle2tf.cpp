#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <base/Angle.h>

void AngleCallback(const base::Angle::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angle2tf");
    ros::NodeHandle Nh;
    ros::Subscriber AngleSubs=Nh.subscribe("Sensors/BodyAngle", 1, &AngleCallback);

    ros::spin();

    return 0;
}

void AngleCallback(const base::Angle::ConstPtr& msg)
{
    geometry_msgs::TransformStamped TFAngleMsg;
    tf2::Quaternion q;
    tf2_ros::TransformBroadcaster TF;

    TFAngleMsg.header.seq=msg->header.seq;
    TFAngleMsg.child_frame_id="jointRear";
    TFAngleMsg.header.frame_id="jointFront";
    TFAngleMsg.header.stamp=msg->header.stamp;


    q.setRPY(0,0,msg->angle);
    TFAngleMsg.transform.translation.x=0;
    TFAngleMsg.transform.translation.y=0;
    TFAngleMsg.transform.translation.z=0;

    TFAngleMsg.transform.rotation.x=q.x();
    TFAngleMsg.transform.rotation.y=q.y();
    TFAngleMsg.transform.rotation.z=q.z();
    TFAngleMsg.transform.rotation.w=q.w();

    TF.sendTransform(TFAngleMsg);


}
