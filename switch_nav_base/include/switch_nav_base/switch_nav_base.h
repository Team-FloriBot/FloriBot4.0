#ifndef SWITCH_NAV_BASE_H
#define SWITCH_NAV_BASE_H

#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib_msgs/GoalStatus.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ros/console.h>

class SwitchNavBase
{
    public:
        SwitchNavBase();
        SwitchNavBase(ros::NodeHandle* nh);
        ~SwitchNavBase();

    private:
        void create_sub_pub_();
        void cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr& msg);
        void goal_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void goal_status_callback_(const actionlib_msgs::GoalStatus::ConstPtr& msg);
        void tf_callback(const ros::TimerEvent& e);
        uint32_t seq_;
        uint8_t goal_status_;
        
        double linear_speed_;
        std::string frame_id_front_;
        std::string frame_id_rear_;
        std::string frame_id_output_;
        std::string frame_id_map_;
        std::string frame_id_parent_;
        ros::NodeHandle* nh_;
        ros::Timer tf_timer_;
        ros::Subscriber cmd_vel_subscriber_;
        geometry_msgs::PoseStamped goal_pose_in_map_;
        geometry_msgs::PoseStamped goal_pose_in_front_;
        geometry_msgs::TransformStamped tf_nav_base_default_;
        geometry_msgs::TransformStamped tf_nav_base_;
        geometry_msgs::TransformStamped tf_rear2front_;
        geometry_msgs::TransformStamped tf_map2front_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener* ptr_tf_listener_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
};
#endif