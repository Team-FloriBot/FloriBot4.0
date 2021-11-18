#ifndef SWITCH_BASE_LINK_H
#define SWITCH_BASE_LINK_H

#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <ros/console.h>

class SwitchBaseLink
{
    public:
        SwitchBaseLink(ros::NodeHandle* nh);
        ~SwitchBaseLink();

    private:
        void create_sub_pub_();
        void cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr& msg);
        void tf_callback(const ros::TimerEvent& e);
        unsigned int seq_;
        double linear_speed_;
        std::string topic_sub_;
        std::string frame_id_input1_;
        std::string frame_id_input2_;
        std::string frame_id_output_;
        std::string frame_id_parent_;
        ros::NodeHandle* nh_;
        ros::Timer tf_timer_;
        ros::Subscriber cmd_vel_subscriber_;
        geometry_msgs::TransformStamped tf_stamped_;
        tf2_ros::TransformBroadcaster base_link_broadcaster_;
};
#endif